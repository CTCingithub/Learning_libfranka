#ifndef TOEIGEN_H
#define TOEIGEN_H

#include <Eigen/Dense>

Eigen::Matrix4d GetTransMat_4x4(const std::array<double, 16> &TransMat);
Eigen::Matrix<double, 7, 7> GetMat_7x7(const std::array<double, 49> &Mat);

#endif