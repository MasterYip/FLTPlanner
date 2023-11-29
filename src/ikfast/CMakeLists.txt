# FILEPATH: /home/yip/CodeSpace/Fast-Legged-Planner_ws/flplanner_ws/src/fast_legged_planner/src/ikfast/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)

# Add any additional compiler flags here
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(IKFast)

# Add your source files here
set(SOURCES
    test_leg.cpp
)

# Add any additional include directories here
message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
include_directories(
    ${PROJECT_SOURCE_DIR}
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

