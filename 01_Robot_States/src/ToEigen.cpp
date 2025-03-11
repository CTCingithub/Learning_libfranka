#include <Eigen/Dense>

Eigen::Matrix4d GetTransMat_4x4(std::array<double, 16> TransMat)
{
    return Eigen::Map<Eigen::Matrix4d>(TransMat.data());
}