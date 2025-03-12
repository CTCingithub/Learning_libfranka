#include <iostream>
#include <boost/circular_buffer.hpp>

#include "Eigen/Core"

#include "PrintCache.h"

int main(int argc, char **argv)
{
    boost::circular_buffer<int> cache(3);
    std::cout << "Create a circular buffer with 3 int values" << std::endl;
    std::cout << "Current Cache size: " << cache.size() << std::endl;
    cache.push_back(1);
    std::cout << "Push back 1" << std::endl;
    std::cout << "Current Cache size: " << cache.size() << std::endl;
    std::cout << "Current Cache capacity: " << cache.capacity() << std::endl;
    std::cout << cache[0] << " " << cache[1] << " " << cache[2] << std::endl;
    cache.push_back(2);
    cache.push_back(3);
    std::cout << "Push back 2 3" << std::endl;
    std::cout << "Current Cache size: " << cache.size() << std::endl;
    std::cout << "Current Cache capacity: " << cache.capacity() << std::endl;
    std::cout << cache[0] << " " << cache[1] << " " << cache[2] << std::endl;
    cache.push_back(4);
    std::cout << "Push back 4" << std::endl;
    std::cout << cache[0] << " " << cache[1] << " " << cache[2] << std::endl;

    boost::circular_buffer<Eigen::Vector3d> cache_eigen(3);
    std::cout << "Create a circular buffer with 3 Eigen::Vector3d" << std::endl;
    Eigen::Vector3d vec1(0.1, 0.0, 0.0);
    Eigen::Vector3d vec2(0, 0.2, 0.3);
    Eigen::Vector3d vec3(1., 2.5, 3);
    std::cout << "Create Eigen::Vector3d" << std::endl;
    std::cout << "vec1: " << vec1.transpose() << std::endl;
    std::cout << "vec2: " << vec2.transpose() << std::endl;
    std::cout << "vec3: " << vec3.transpose() << std::endl;
    cache_eigen.push_back(vec1);
    std::cout << "Push back vec1:" << std::endl;
    PrintCache(cache_eigen);
    cache_eigen.push_back(vec2);
    cache_eigen.push_back(vec3);
    cache_eigen.push_back(vec3);
    std::cout << "Push back vec2, vec3, vec3:" << std::endl;
    PrintCache(cache_eigen);

    return 0;
}