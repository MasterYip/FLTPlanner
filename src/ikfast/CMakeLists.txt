# FILEPATH: /home/yip/CodeSpace/Fast-Legged-Planner_ws/flplanner_ws/src/fast_legged_planner/src/ikfast/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
add_compile_options(-O3 -Wall -shared -fPIC)
# add_compile_options(-O2 -Wall -Wextra -Wpedantic -fPIC)
project(IKFast)


set(SOURCES
    ikfast_interface.cpp
    ikfast_gen.cpp
)

include_directories(
    ${PROJECT_SOURCE_DIR}
)

# IKFast Library
add_library(${PROJECT_NAME} ${SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE -DIKFAST_NO_MAIN)

# Python Bindings
find_package(pybind11)
pybind11_add_module(pyikfast SHARED ik_bindings.cpp)
target_link_libraries(pyikfast PRIVATE ${PROJECT_NAME})
# add_library(pyikfast MODULE ik_bindings.cpp)
# target_link_libraries(pyikfast PRIVATE pybind11::module)

# Test
add_executable(test_ik test_ik.cpp)
target_link_libraries(test_ik PRIVATE ${PROJECT_NAME})
