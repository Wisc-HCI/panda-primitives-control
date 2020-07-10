#ifndef PANDA_KINEMATICS_H
#define PANDA_KINEMATICS_H
#include <array>
#include <Eigen/Core>
#include <vector>

namespace PandaController {
    namespace Kinematics {
        const std::vector<std::array<double, 3>> PandaGripperDHA{
            {       0,   0.333,       0},
            {       0,       0, -M_PI/2},
            {       0, 0.31599,  M_PI/2},
            { 0.08249,       0,  M_PI/2},
            {-0.08249,   0.384, -M_PI/2},
            {       0,       0,  M_PI/2},
            {  0.0879,       0,  M_PI/2},
            {       0,  0.1069,       0}
        };
    }
    Eigen::Matrix<double, 4, 4> EEFromDHA(std::array<double, 7> q, std::vector<std::array<double, 3>> dha);
    std::array<double, 42> jacobianFromDHA(std::array<double, 7> q, std::vector<std::array<double, 3>> dha);
}
#endif