#ifndef TOEIGEN_H
#define TOEIGEN_H

#include <Eigen/Dense>

Eigen::Matrix4d GetTransMat_4x4(std::array<double, 16> TransMat);

#endif