cmake_minimum_required(VERSION 2.8.3)

project(gcs_path_search)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
# --copy-dt-needed-entries
# set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--copy-dt-needed-entries")

add_subdirectory(gcs_traj_opt)

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
  )


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    include
)


# Dynamic reconfigure
set (CVX_DYNCFG_PATH config/cvx_trajopt.cfg)
execute_process(COMMAND chmod a+x ${CVX_DYNCFG_PATH}
      		  	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      		  	RESULT_VARIABLE cmd_result
      		 	OUTPUT_VARIABLE cmd_ver)
message(STATUS "Chmod a+x the dynamic_reconfigure file")

generate_dynamic_reconfigure_options(
        ${CVX_DYNCFG_PATH}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# Executables

add_executable(polyve_node 
  src/polyve_node.cpp
  src/polyve.cpp)
target_link_libraries(polyve_node
  ${catkin_LIBRARIES}
  gcs_traj_opt)


add_executable(cvx_trajopt_node
  src/cvx_trajopt_node.cpp
  src/cvx_trajopt.cpp)

add_dependencies(cvx_trajopt_node ${PROJECT_NAME}_gencfg)
target_link_libraries(cvx_trajopt_node
  ${catkin_LIBRARIES} 
  gcs_traj_opt) 
# FIXME: gmp and mpfr must be linked manually


