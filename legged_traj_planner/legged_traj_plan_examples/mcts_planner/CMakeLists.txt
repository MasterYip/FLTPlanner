cmake_minimum_required(VERSION 2.6)
project(mcts_planner)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -fPIC")

# MCTS Config File
set(MCTS_CFG_FILE ${PROJECT_SOURCE_DIR}/config.txt)
configure_file(${PROJECT_SOURCE_DIR}/include/config.h.in ${PROJECT_SOURCE_DIR}/include/config.h)

# add_compile_options(-Wall -Wextra -Wpedantic) 是 CMake 的一个命令，用于向编译器添加编译选项。这个命令会将 -Wall、-Wextra 和 -Wpedantic 三个编译选项添加到编译器的命令行参数中。
# 这三个编译选项都是用于启用编译器的警告功能，具体含义如下：
# -Wall：启用所有常见的警告选项，包括一些常见的编程错误。
# -Wextra：启用一些额外的警告选项，包括一些更加严格的检查。
# -Wpedantic：启用严格的 C++ 标准检查，对于不符合标准的代码会给出警告。
# 通过添加这些编译选项，可以让编译器检测到一些潜在的编程错误，帮助开发者编写更加健壮的代码。但需要注意的是，有些编译选项可能会产生一些误报，因此需要开发者根据实际情况决定是否启用它们。
add_compile_options(-Wall -Wextra -Wpedantic)

# CMAKE_EXPORT_COMPILE_COMMANDS 是 CMake 中的一个选项，用于生成编译命令数据库（Compilation Database）。
# 这个数据库记录了每个源文件编译所需的编译器命令，包括编译器名称、选项、输入文件和输出文件等信息。
# 生成的数据库可以用于各种代码编辑器和工具，如 Vim、Emacs、VSCode、clangd 等，用于代码导航、代码补全、语法检查、重构等操作。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()


# Find the grid_map package
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_sdf REQUIRED)

# 包含 MyLibrary 的头文件路径
include_directories(${grid_map_msgs_INCLUDE_DIRS})
include_directories(${grid_map_core_INCLUDE_DIRS})



# GLPK路径
set(GLPK_INCLUDE_DIR "/usr/local/include")
set(GLPK_LIBRARY_DIR "/usr/local/lib")
include_directories("${GLPK_INCLUDE_DIR}")	#包含GLPK头文件
link_directories("${GLPK_LIBRARY_DIR}")			#GLPK的链接库


# cdd库
set(CDD_INCLUDE_DIR "/usr/local/include/cddlib")
set(CDD_LIBRARY_DIR "/usr/local/lib/libcdd.so")
include_directories("${CDD_INCLUDE_DIR}")	  # 包含cdd的头文件
link_directories("${CDD_LIBRARY_DIR}")			# cdd库的链接库

find_package(pinocchio REQUIRED)


# IK_FAST 
# 添加自定义头文件
include_directories(${PROJECT_SOURCE_DIR}/IKfast/IK_Fast_forHexMini)
# # 添加库文件
add_subdirectory (${PROJECT_SOURCE_DIR}/IKfast/IK_Fast_forHexMini) # IK_FAST库


###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)


set(lib_RST_headers
    include/constrains/Bretl.hh
    include/constrains/solve_LP_GLPK.hh
    include/constrains/util.hh
    # constrains/solver_Gurobi.cpp
    include/constrains/my_cdd.hh
    include/constrains/dynamic_constrain.hh
)

set(lib_RST_sources
    ${lib_RST_headers}
    constrains/Bretl.cpp
    constrains/solve_LP_GLPK.cpp
    # constrains/solver_Gurobi.cpp
    constrains/my_cdd.cpp
    constrains/util.cpp
    constrains/dynamic_constrain.cpp
    constrains/dynamic_constrain_pinnochio.cpp
)
set(lib_RST ConstrainsLib)
add_library(${lib_RST} SHARED ${lib_RST_sources})		#构建  机器人状态转移  动态链接库
target_link_libraries(${lib_RST} ${lib_Bisic} libglpk.so  ${CDD_LIBRARY_DIR} ${catkin_LIBRARIES} pinocchio::pinocchio IK_FAST)  #链接外部库

add_library(${PROJECT_NAME}
    planning/geometry.cpp
    planning/myDataType.cpp
    planning/planning.cpp
    planning/zobrist_hash.cpp
    planning/search_tree.cpp
    planning/HexapodParameter.cpp
    planning/user.cpp
    planning/saveHashTable.cpp
    planning/collisionCheck.cpp
    constrains/kinematics_constrain.cpp
    planning/contactPlannerInterface.cpp
    ForwardKinematic/HexMiniFK.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${grid_map_core_LIBRARIES}
  ${grid_map_sdf_LIBRARIES}
  ${CDD_LIBRARY_DIR}
  ${lib_RST}
  ${pinocchio_LIBRARIES}
)

# Export headers for this library to other libraries
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# 设置库文件的输出路径
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)