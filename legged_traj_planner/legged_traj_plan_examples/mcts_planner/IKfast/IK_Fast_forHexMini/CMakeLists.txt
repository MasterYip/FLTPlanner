cmake_minimum_required(VERSION 3.10)
project(IK_FAST)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# include_directories(.)
include_directories(
    ${PROJECT_SOURCE_DIR}
)


set(SOURCES
    ikfast_interface_RB.cpp
    solver_RB.cpp
    ikfast_interface_RF.cpp
    solver_RF.cpp
    ikfast_interface_LB.cpp
    solver_LB.cpp
    ikfast_interface_LF.cpp
    solver_LF.cpp
    HexMini_IK_interface.cpp
)
# include_directories(
#     ${PROJECT_SOURCE_DIR}
# )

# IKFast Library
add_library(${PROJECT_NAME} ${SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE -DIKFAST_NO_MAIN)

# add_executable(solver_RF solver_RF.cpp)
# add_executable(solver_RB solver_RB.cpp)

# Test
# add_executable(test_ik test_ik.cpp)
# target_link_libraries(test_ik PRIVATE ${PROJECT_NAME})