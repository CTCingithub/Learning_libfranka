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
find_package(Threads REQUIRED)

# Build shared library


# Build executable
add_executable(01_hello_world examples/01_hello_world.cpp)
target_link_libraries(01_hello_world
    Threads::Threads
)

add_executable(02_wrong_output examples/02_wrong_output.cpp)
target_link_libraries(02_wrong_output
    Threads::Threads
)

add_executable(03_sleeping examples/03_sleeping.cpp)
target_link_libraries(03_sleeping
    Threads::Threads
)

add_executable(04_use_mutex examples/04_use_mutex.cpp)
target_link_libraries(04_use_mutex
    Threads::Threads
)