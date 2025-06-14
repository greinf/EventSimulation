cmake_minimum_required(VERSION 3.16)
project(EventSimulation)

# Use modern C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable folders in Visual Studio
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Find vcpkg packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executable target first!
add_executable(EventSimulation
  src/main.cpp
  src/Surface.cpp
  src/Plance.cpp
  src/Object.cpp
)

# Then link libraries to it
target_link_libraries(EventSimulation PRIVATE
  ${OpenCV_LIBS}
  Eigen3::Eigen
)