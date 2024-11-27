cmake_minimum_required(VERSION 3.10)

# Project name
project(EpistemicPlanning)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
if (APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")




# Find the Eigen3 package (header-only, so no linking required)
find_path(EIGEN3_INCLUDE_DIR Eigen/Dense REQUIRED PATHS "${PROJECT_SOURCE_DIR}/eigen-master")

# Add Eigen3 include directory
include_directories(${EIGEN3_INCLUDE_DIR})

# Include other source folders if necessary (e.g., 'include' folder in your project)
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}")

# Add the 'mdutec planner' folder to the include directories for headers
include_directories("${PROJECT_SOURCE_DIR}/mdutec_planner/")

# Add the path to the library files
link_directories("${PROJECT_SOURCE_DIR}/mdutec_planner/lib")

# Add the executable target (list all your source files)
add_executable(epistemic_planning
    "src/main.cpp"
    "src/algo.cpp"
    "src/planning.cpp"
    "src/robot.cpp"
    "src/readProblem.cpp"
    "src/MCTS_MTSP_EP.cpp"
)

# Link the static library 'libmdutec.a'
 target_link_libraries(epistemic_planning PRIVATE mdutec)
# target_link_libraries(epistemic_planning PRIVATE stdc++fs)

# Optional: set build type (Debug, Release)
set(CMAKE_BUILD_TYPE Debug)  # or Debug

