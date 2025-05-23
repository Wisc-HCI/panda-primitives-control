#ifndef PANDA_KINEMATICS_H
#define PANDA_KINEMATICS_H
#include <array>
#include <Eigen/Core>
#include <vector>
#include "DHA.h"

namespace PandaController {
    Eigen::Matrix<double, 4, 4> EEFromDHA(std::array<double, 7> q, std::vector<DHA> dha, Eigen::Matrix<double, 4, 4> ee_link);
    std::array<double, 42> jacobianFromDHA(std::array<double, 7> q, std::vector<DHA> dha, Eigen::Matrix<double, 4, 4> ee_link);
    Eigen::VectorXd getJointAngles(std::array<double, 7> q_start, std::vector<DHA> dha, Eigen::Matrix<double, 4, 4> ee_link, Eigen::Vector3d ee_pos, Eigen::Vector3d ee_angles);
}
#endif