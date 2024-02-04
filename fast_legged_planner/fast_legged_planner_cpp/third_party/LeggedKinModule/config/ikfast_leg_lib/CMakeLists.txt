cmake_minimum_required(VERSION 3.0.2)

project(ikfast_leg_lib)


# add_compile_options(-O3 -Wall -shared -fPIC -Wno-unused-variable)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if(1)
    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIRS})
endif()
include_directories(
    ${PROJECT_SOURCE_DIR}
)

add_library(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
file(GLOB ARCHIVE_LIBS ${PROJECT_SOURCE_DIR}/*.a)
target_link_libraries(${PROJECT_NAME} PUBLIC ${ARCHIVE_LIBS})
if(1)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})
endif()

add_executable(test_${PROJECT_NAME} test.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
