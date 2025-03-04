// -*- coding: utf-8 -*-
#ifndef HexMini_IK_interface_
#define HexMini_IK_interface_
#include "Eigen/Dense"


    /**
     * 机器人逆运动学 
     * @param x_W 机器人位置x
     * @param y_W 机器人位置y
     * @param z_W 机器人位置z
     * @param roll 机器人姿态roll
     * @param pitch 机器人姿态pitch
     * @param yaw 机器人姿态yaw
     * @param feetPosition_W 机器人六条腿的末端位置 - 注意参考坐标系为世界坐标系
     * @return 机器人关节角度 - 注意顺序为RF, RM, RB, LF, LM, LB
    */
    std::pair<bool, Eigen::Matrix<double, Eigen::Dynamic, 1>> wholeBodyIK(float x_W, float y_W, float z_W,float roll, 
        float pitch, float yaw,  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> feetPosition_W);

    std::pair<bool, Eigen::Matrix<double, Eigen::Dynamic, 1>> leg_IK(float x_W, float y_W, float z_W,float roll, 
        float pitch, float yaw,  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> feetPosition_W, int legIndex);


#endif