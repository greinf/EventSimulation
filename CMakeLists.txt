cmake_minimum_required(VERSION 3.10)
project(EventSimulation)

set(CMAKE_CXX_STANDARD 17)

# Use OpenCV
find_package(OpenCV REQUIRED)

# Include headers (for future modularization)
include_directories(include)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Executable
add_executable(EventSimulation src/main.cpp)

# Link OpenCV
target_link_libraries(EventSimulation PRIVATE ${OpenCV_LIBS})
