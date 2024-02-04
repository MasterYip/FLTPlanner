cmake_minimum_required(VERSION 3.0.2)
project(fast_legged_planner_cpp)

add_compile_options(-std=c++11)

# Settings
set(GTEST "ON")

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Dependancy
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
# FIXME: you should split the project down to smaller packages (for decoupling)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  grid_map_core)


file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE TESTS "test/*.cpp")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PINOCCHIO_LIBRARIES} ${catkin_LIBRARIES})

# Tests
# foreach(test ${TESTS})
#   get_filename_component(test_name ${test} NAME_WE)
#   add_executable(${test_name} ${test})
#   target_link_libraries(${test_name} ${PROJECT_NAME})
# endforeach()

# GTest
if(GTEST)
  find_package(MathGL REQUIRED)
  find_package(GTest REQUIRED)  
  add_executable(flplanner_tests ${TESTS})
  # set(URDF_PATH ${PROJECT_SOURCE_DIR}/../)
  target_include_directories(flplanner_tests
    PRIVATE
    ${GTEST_INCLUDE_DIRS}
    ${MathGL_INCLUDE_DIRS}
    test)
  target_link_libraries(flplanner_tests ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread ${MathGL_LIBRARIES})
endif()
