cmake_minimum_required(VERSION 3.0.2)

project(IKFast)

add_compile_options(-O3 -Wall -shared -fPIC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
# add_compile_options(-O2 -Wall -Wextra -Wpedantic -fPIC)

## Settings
set(PYBIND_NAME pyikfast_el_mini_back) # Change this to the name of your python module
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
# configure_file(config.h.in ${CMAKE_BINARY_DIR}/config.h @ONLY)
configure_file(config.h.in ${CMAKE_SOURCE_DIR}/config.h @ONLY)
pybind11_add_module(${PYBIND_NAME} SHARED ik_bindings.cpp)
target_link_libraries(${PYBIND_NAME} PRIVATE ${PROJECT_NAME})

# Test
add_executable(test_ik test_ik.cpp)
target_link_libraries(test_ik PRIVATE ${PROJECT_NAME})
