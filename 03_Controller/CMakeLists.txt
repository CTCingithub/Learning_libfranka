cmake_minimum_required(VERSION 3.5)

# Avoid std::optional not declared error
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

project(MultiThread)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Add include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Load dependencies
if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)

# Build shared library
add_library(read-config-yaml SHARED src/ReadConfig.cpp)
target_link_libraries(read-config-yaml
    yaml-cpp::yaml-cpp
)

add_library(motion-gen SHARED src/MotionGenerator.cpp)
target_link_libraries(motion-gen
    Eigen3::Eigen
    Franka::Franka
)

add_library(cosine-wave-gen SHARED src/CosineWaveGenerator.cpp)
target_link_libraries(cosine-wave-gen
    Eigen3::Eigen
    Franka::Franka
)

add_library(polynomial-gen SHARED src/PolynomialMotionGenerator.cpp)
target_link_libraries(polynomial-gen
    Eigen3::Eigen
    Franka::Franka
)

add_library(controllers SHARED src/Controllers.cpp)
target_link_libraries(controllers
    Eigen3::Eigen
    Franka::Franka
)

add_library(default-behavior SHARED src/DefaultBehavior.cpp)
target_link_libraries(default-behavior
    Franka::Franka
    motion-gen
)


# Build executable
add_executable(ToInitialPose src/ToInitialPose.cpp)
target_link_libraries(ToInitialPose
    Franka::Franka
    read-config-yaml
    default-behavior
)

add_executable(SwitchMotion src/SwitchMotion.cpp)
target_link_libraries(SwitchMotion
    Franka::Franka
    read-config-yaml
    default-behavior
    cosine-wave-gen
    polynomial-gen
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
    spdlog::spdlog
)

add_executable(00_point2point examples/00_point2point.cpp)
target_link_libraries(00_point2point
    Franka::Franka
    read-config-yaml
    motion-gen
    default-behavior
)

add_executable(01_cartesian_impedance_control examples/01_cartesian_impedance_control.cpp)
target_link_libraries(01_cartesian_impedance_control
    Franka::Franka
    read-config-yaml
    motion-gen
    default-behavior
)

add_executable(02_sampling_control examples/02_sampling_control.cpp)
target_link_libraries(02_sampling_control
    Franka::Franka
    Eigen3::Eigen
    read-config-yaml
    default-behavior
    controllers
    cosine-wave-gen
)

add_executable(03_delayed_control examples/03_delayed_control.cpp)
target_link_libraries(03_delayed_control
    Franka::Franka
    Eigen3::Eigen
    read-config-yaml
    default-behavior
    controllers
    cosine-wave-gen
)