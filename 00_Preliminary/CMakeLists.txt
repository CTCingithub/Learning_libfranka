cmake_minimum_required(VERSION 3.5)

# Avoid std::optional not declared error
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

project(Preliminary)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# Add include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Load dependencies
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)

# Build shared library
add_library(data-logger SHARED src/DataLogger.cpp)
target_link_libraries(data-logger
    spdlog::spdlog
)

# Build executable
add_executable(01_try-catch examples/01_try-catch.cpp)

add_executable(02_circular_buffer examples/02_circular_buffer.cpp)
target_link_libraries(02_circular_buffer
    Eigen3::Eigen
)

add_executable(03_multi_thread_wrong_output examples/03_multi_thread_wrong_output.cpp)
target_link_libraries(03_multi_thread_wrong_output
    Threads::Threads
)

add_executable(05_spdlog examples/05_spdlog.cpp)
target_link_libraries(05_spdlog
    spdlog::spdlog
    data-logger
)