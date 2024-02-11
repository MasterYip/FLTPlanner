cmake_minimum_required(VERSION 3.0.2)
project(gcs_traj_opt)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)

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
  ${CGAL_LIBRARIES}
)

# export(PACKAGE ${PROJECT_NAME})
