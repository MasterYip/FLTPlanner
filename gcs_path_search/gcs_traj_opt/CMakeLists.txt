cmake_minimum_required(VERSION 3.0.2)
project(gcs_traj_opt)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
# disable -Wclass-memaccess
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC -Wno-class-memaccess")

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(grid_map_core REQUIRED)

file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${CGAL_INCLUDE_DIRS}
  INTERFACE
  ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME}
  ${grid_map_core_LIBRARIES}
  ${CGAL_LIBRARIES}
)

# Export as a package
# export(PACKAGE ${PROJECT_NAME})
# export(EXPORT ${PROJECT_NAME})