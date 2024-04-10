cmake_minimum_required(VERSION 3.16)
project(third_party)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
    INCLUDE_DIRS 
        LeggedKinModule/core
        LeggedKinModule/config
        LeggedKinModule/config/ikfast_leg_lib
    LIBRARIES LeggedKinModule
    CATKIN_DEPENDS
    DEPENDS
)

# disable warnings in third-party libraries
add_definitions(-w)

add_subdirectory(LeggedKinModule)

remove_definitions(-w)
