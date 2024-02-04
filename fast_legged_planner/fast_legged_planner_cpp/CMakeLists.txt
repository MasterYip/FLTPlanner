cmake_minimum_required(VERSION 3.0.2)
project(fast_legged_planner_cpp)

add_compile_options(-std=c++11)

# Settings
set(USE_GTEST "ON")
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Dependancy
add_subdirectory(third_party/LeggedKinModule)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(catkin REQUIRED
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_sdf)

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE TESTS "test/test_*.cpp")
file(GLOB_RECURSE GTESTS "test/gtest_*.cpp")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
# FIXME: this project depend on catkin, but it is not a catkin package
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PINOCCHIO_LIBRARIES} ${catkin_LIBRARIES} LeggedKinModule)

# GTest
if(USE_GTEST)
  find_package(MathGL REQUIRED)
  find_package(GTest REQUIRED)  
  # Configure file
  set(TEST_URDF_PATH ${PROJECT_SOURCE_DIR}/../../hexapod_robot_assets/model/elspider_air/urdf/el_mini.urdf)
  set(TEST_PKG_DIRS ${PROJECT_SOURCE_DIR}/../../hexapod_robot_assets/model/elspider_air)
  configure_file(${PROJECT_SOURCE_DIR}/test/urdf_path.h.in ${PROJECT_SOURCE_DIR}/test/urdf_path.h @ONLY)
  # Build
  add_executable(flplanner_tests ${GTESTS})
  target_include_directories(flplanner_tests
    PRIVATE
    ${GTEST_INCLUDE_DIRS}
    ${MathGL_INCLUDE_DIRS}
    test)
  target_link_libraries(flplanner_tests ${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread ${MathGL_LIBRARIES})
endif()

# Tests
foreach(test ${TESTS})
  get_filename_component(test_name ${test} NAME_WE)
  add_executable(${test_name} ${test})
  target_link_libraries(${test_name} ${PROJECT_NAME})
endforeach()