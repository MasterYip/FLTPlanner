/**
 * @file vmc_controller.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "hexapod_controller/vmc_controller.hpp"

VMCController::VMCController(ros::NodeHandle &nh)
{
    foot_cmd_sub_ = nh.subscribe("/foot_state_sub", 1, &VMCController::footStateCallback, this);
}

VMCController::footStateCallback(const hexapod_controller::FootState &msg)
{
    foot_state_ = msg;
}