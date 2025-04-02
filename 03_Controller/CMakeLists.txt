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

add_library(default-behavior SHARED src/DefaultBehavior.cpp)
target_link_libraries(default-behavior
    Franka::Franka
)


# Build executable
add_executable(01_cartesian_impedance_control examples/01_cartesian_impedance_control.cpp)
target_link_libraries(01_cartesian_impedance_control
    Franka::Franka
    read-config-yaml
    motion-gen
    default-behavior
)
