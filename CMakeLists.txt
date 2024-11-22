cmake_minimum_required(VERSION 3.10)

# Project name
project(EpistemicPlanning)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the Eigen3 package (header-only, so no linking required)
# Assuming Eigen is in the 'eigen-master' folder
find_path(EIGEN3_INCLUDE_DIR Eigen/Dense REQUIRED PATHS "${PROJECT_SOURCE_DIR}/eigen-master")

# Add Eigen3 include directory
include_directories(${EIGEN3_INCLUDE_DIR})

# Include other source folders if necessary (e.g., 'include' folder in your project)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add the executable target (list all your source files)
add_executable(epistemic_planning
    src/main.cpp
    src/algorithm.cpp
    src/planning.cpp
    src/robot.cpp
    # List other source files here as needed
)

# Optional: set build type (Debug, Release)
set(CMAKE_BUILD_TYPE Release)  # or Debug
