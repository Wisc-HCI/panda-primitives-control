#include "Kinematics.h"
#include <array>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "DHA.h"
#include <iostream>
#include "nlopt.hpp"
#include <functional>
#include <chrono>
#include <thread>
#include "PandaController.h"
using namespace std;

namespace PandaController {

    namespace {
        auto op = nlopt::opt(nlopt::LN_NEWUOA, 7);
        class CostFunction {
            std::function<double (const std::vector<double>&, std::vector<double>&)> f;
        public:
            CostFunction(std::function<double (const std::vector<double>&, std::vector<double>&)> f) {
                this->f = f;
            }

            static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) {
                return (*reinterpret_cast<CostFunction*>(data))(x, grad); 
            }

            double operator()(const std::vector<double> &x, std::vector<double> &grad) {
                return this->f(x, grad);
            }
        };
    }

    Eigen::Matrix<double, 4, 4> EEFromDHA(array<double, 7> q, vector<DHA> dha, Eigen::Matrix<double, 4, 4> ee_link) {
        // https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
        
        Eigen::Matrix<double, 4, 4> ee_trans = Eigen::MatrixXd::Identity(4,4);
        for (int i = 0; i < dha.size(); i++) {
            ee_trans = ee_trans * dha[i].to_matrix(q);
        }
        return ee_trans * ee_link;
    }

    array<double, 42> jacobianFromDHA(array<double, 7> q, vector<DHA> dha, Eigen::Matrix<double, 4, 4> ee_link) {
        vector<Eigen::Matrix4d> transforms = vector<Eigen::Matrix4d>();
        vector<Eigen::Matrix4d> transforms_derivative = vector<Eigen::Matrix4d>();
        Eigen::Matrix4d ee_trans = Eigen::MatrixXd::Identity(4,4);
        for (int i = 0; i < dha.size(); i++) {
            auto trans = dha[i].to_matrix(q);
            ee_trans = ee_trans * trans;
            transforms.push_back(trans);
            transforms_derivative.push_back(dha[i].to_matrix_derivative(q));
        }
        auto R = (ee_trans * ee_link).topLeftCorner(3, 3);

        Eigen::Matrix<double, 6, 7> jacobian = Eigen::MatrixXd::Zero(6, 7);
        for (int j = 0; j < 7; j++) {
            Eigen::Matrix4d jac = Eigen::MatrixXd::Identity(4,4);
            for (int i = 0; i < transforms.size(); i++) {
                if (i == j) {
                    jac = jac * transforms_derivative[i];
                } else {
                    jac = jac * transforms[i];
                }
            }
            jac = jac * ee_link;
            jacobian(0, j) = jac(0, 3);
            jacobian(1, j) = jac(1, 3);
            jacobian(2, j) = jac(2, 3);

            auto W = jac.topLeftCorner(3,3) * R.transpose();
            jacobian(3, j) = W(2,1); // w_x
            jacobian(4, j) = W(0,2); // w_y
            jacobian(5, j) = W(1,0); // w_z
        }
        
        array<double, 42> jacobian_array{};
        // Pointer magic
        Eigen::Map<Eigen::MatrixXd>(jacobian_array.data(), 6, 7) = jacobian;
        return jacobian_array;
    }

    double groove_loss(double x, double t, int d, double c, double f, int g) {
        return 1-exp(-pow(x - t, d) / (2 * pow(c, d))) + f * pow(x - t, g);
    }

    double groove_loss_derivative(double x, double t, int d, double c, double f, int g) {
        return (d * pow(x - t, d-1) / (2 * pow(c, d))) * exp(-pow(x - t, d) / (2 * pow(c, d))) + g * f * pow(x - t, g - 1);
    }


    Eigen::VectorXd getJointAngles(array<double, 7> q_start, vector<DHA> dha, Eigen::Matrix<double, 4, 4> ee_link, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_angles) {
        Eigen::Matrix<double, 2, 7> joint_limits;
        joint_limits << 
            -2.8973, 2.8973,
            -1.7628, 1.7628,
            -2.8973, 2.8973,
            -3.0718,-0.0698,
            -2.8973, 2.8973,
            -0.0175, 3.7525,
            -2.8973, 2.8973;
        
        auto angles = PandaController::EulerAngles();
        angles.roll = ee_angles[0]; angles.pitch = ee_angles[1]; angles.yaw = ee_angles[2];
        auto ee_orientation = eulerToQuaternion(angles);
        auto cost_func = CostFunction([=](const std::vector<double> &q, std::vector<double> &grad) -> double {
            std::this_thread::sleep_for (std::chrono::microseconds(1)); // So we can yield to the franka thread every now and then.
            for (int i = 0; i < grad.size(); i++) grad[i] = 0;

            double c = 0;
            for (int i = 0; i < q.size(); i++){
                double t = (q[i] - joint_limits(i,0)) / (joint_limits(i,1) - joint_limits(i,0));
                t = 2 * t - 1;
                c += 0.05 * pow(t/0.85, 50);
            }

            auto q_arr = array<double,7>();
            for (int i = 0; i < 7; i++) q_arr[i] = q[i];
            auto T =  Eigen::Affine3d(EEFromDHA(q_arr, dha, ee_link));
            auto pos = T.translation();
            auto orientation = Eigen::Quaterniond(T.linear()).normalized();
            double xd = (pos - ee_pos).norm();
            double ad = orientation.angularDistance(ee_orientation);
            //xd = xd * xd; ad = ad * ad;
            c += 10 * groove_loss(xd, 0, 2, 0.01, 10, 2);
            c += 10 * groove_loss(ad, 0, 2, 0.01, 10, 2);
            // if (grad.size() > 0) {
            //     Eigen::Matrix<double, 6, 7> J = Eigen::Map<Eigen::Matrix<double, 6 ,7>>(jacobianFromDHA(q_arr, dha, ee_link).data(), 6, 7);
            //     Eigen::VectorXd grad_v(7);
            //     grad_v.setZero();
            //     grad_v += 2 * J.transpose().leftCols(3) * (pos - ee_pos) * groove_loss_derivative(xd, 0, 2, 0.1, 10, 2);
            //     grad_v += 2 * J.transpose().rightCols(3) * (euler - ee_angles) * groove_loss_derivative(ad, 0, 2, 0.1, 10, 2);
            //     for (int i = 0; i < 7; i++) grad[i] = grad_v[i];
            // }
            return c;
        });

        op.set_min_objective(CostFunction::wrap, &cost_func);
        op.set_xtol_abs(0.001);
        //op.set_ftol_abs(0.00001);
        //op.set_stopval(1e-7);
        //op.set_maxtime(0.005);
        //op.set_xtol_abs(0.00001);
        auto q_vec = vector<double>();
        for (int i = 0; i < 7; i++) q_vec.push_back(q_start[i]);

        auto now = chrono::system_clock::now();
        auto result = op.optimize(q_vec);
        //cout << chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now() - now).count() << endl;
        auto q_arr = array<double,7>();
        for (int i = 0; i < 7; i++) q_arr[i] = result[i];
        auto T =  Eigen::Affine3d(EEFromDHA(q_arr, dha, ee_link));
        auto pos = T.translation();
        auto orientation = Eigen::Quaterniond(T.linear()).normalized();

        return Eigen::Map<Eigen::VectorXd>(result.data(), 7);
    }
}