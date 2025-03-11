#include <Eigen/Dense>

Eigen::Matrix4d GetTransMat_4x4(const std::array<double, 16> &TransMat)
{
    return Eigen::Map<const Eigen::Matrix4d>(TransMat.data());
}