#include <Eigen/Dense>

Eigen::Matrix4d GetTransMat_4x4(const std::array<double, 16> &TransMat)
{
    return Eigen::Map<const Eigen::Matrix4d>(TransMat.data());
}

Eigen::Matrix<double, 7, 7> GetMat_7x7(const std::array<double, 49> &Mat)
{
    return Eigen::Map<const Eigen::Matrix<double, 7, 7>>(Mat.data());
}