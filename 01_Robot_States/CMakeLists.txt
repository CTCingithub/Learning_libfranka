cmake_minimum_required(VERSION 3.4)

# Avoid std::optional not declared error
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

project(FrankaRobotStates)
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
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Build shared library
add_library(read-config-yaml SHARED src/ReadConfig.cpp)
target_link_libraries(read-config-yaml
    yaml-cpp::yaml-cpp
)
add_library(toEigen SHARED src/ToEigen.cpp)
target_link_libraries(toEigen
    Eigen3::Eigen
)
add_library(motion-gen SHARED src/MotionGenerator.cpp)
target_link_libraries(motion-gen
    Eigen3::Eigen
    Franka::Franka
)

# Build executable
add_executable(01_access_robot examples/01_access_robot.cpp)
target_link_libraries(01_access_robot
    Franka::Franka
    yaml-cpp::yaml-cpp
)

add_executable(02_frame examples/02_frame.cpp)
target_link_libraries(02_frame
    Franka::Franka
    yaml-cpp::yaml-cpp
    read-config-yaml
    toEigen
)

add_executable(03_realtime_state examples/03_realtime_state.cpp)
target_link_libraries(03_realtime_state
    Franka::Franka
    yaml-cpp::yaml-cpp
    motion-gen
    read-config-yaml
    toEigen
)