# contactPlanner_HexMIni

步态模式选择：

    在config文件里
    # 步态模式 0:全步态 1:删除相邻腿作为摆动腿的步态（更稳定）
    GaitMode = 1

---
相关阈值设置

    在user.h里
    const float onlyRotateThreshold = 0.05; // 旋转阈值
    const float norminalTrunkHeight = 0.24; // 默认机体高度 
    const float staticStabilityMarginThreshold = 0.05; // 静态稳定阈值


# how to test  consuming time of this program
1. add "set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -pg  -O2")" to cmakeList.txt
2. in , you need to add this code "# add_subdirectory (${PROJECT_SOURCE_DIR}/IKfast/IK_Fast_forHexMini) # IK_FAST库"
3. mkdir build, cd build, cmake .., make .  
4. run the executable cpp
5. gprof ./test gmon.out > analysis.txt

