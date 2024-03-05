cmake_minimum_required(VERSION 2.8.3)

project(gcs_path_search)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
# --copy-dt-needed-entries
# set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--copy-dt-needed-entries")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  # Grid map
  grid_map_core
  grid_map_ros
  grid_map_msgs
  dynamic_reconfigure
  ros_visualizer
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    include
)

add_subdirectory(gcs_traj_opt)

# Dynamic reconfigure
set (DYNCFG_PATH 
  config/cvx_trajopt.cfg
  config/gcs_example.cfg)
execute_process(COMMAND chmod a+x ${DYNCFG_PATH}
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                RESULT_VARIABLE cmd_result
                OUTPUT_VARIABLE cmd_ver)
message(STATUS "Chmod a+x the dynamic_reconfigure file")

generate_dynamic_reconfigure_options(
  ${DYNCFG_PATH}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# Visualizer
add_library(gcs_visualizer
  src/gcs_visualizer.cpp
)
target_link_libraries(gcs_visualizer
  ${catkin_LIBRARIES}
  gcs_traj_opt
)

# Executables

# Trajectory optimizer
add_executable(cvx_trajopt_node
  src/cvx_trajopt_node.cpp
  src/cvx_trajopt.cpp)

add_dependencies(cvx_trajopt_node ${PROJECT_NAME}_gencfg)
target_link_libraries(cvx_trajopt_node
  ${catkin_LIBRARIES} 
  gcs_traj_opt
  gcs_visualizer)

# GCS Examples
add_executable(gcs_example_node
  src/gcs_example_node.cpp
  src/gcs_example.cpp)

add_dependencies(gcs_example_node ${PROJECT_NAME}_gencfg)
target_link_libraries(gcs_example_node
  ${catkin_LIBRARIES} 
  gcs_traj_opt
  gcs_visualizer)



