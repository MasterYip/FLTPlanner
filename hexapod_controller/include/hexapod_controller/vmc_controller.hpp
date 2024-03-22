/**
 * @file vmc_controller.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/log.hpp>
#include <hexapod_controller/FootCmd.h>
#include <hexapod_controller/FootState.h>
#include <Eigen/Core>

#include <ros/ros.h>
/* internal project header files */

class VMCController
{

private:
    double mass_;
    double gravity_ = 9.8;
    double mu;
    pinocchio::SE3 com_pos_;

    pinocchio::SE3 exp_pos_;

    FootCmd foot_cmd_;
    ros::Subscriber foot_cmd_pub_;
    FootState foot_state_;
    ros::Publisher foot_state_sub_;


public:
    VMCController(ros::NodeHandle &nh);
    void footStateCallback(const hexapod_controller::FootState &msg);
    bool getExpWrench(const pinocchio::SE3 &com_pos,
                      const pinocchio::SE3 &exp_pos,
                      pinocchio::Force &exp_wrench);
    bool getGroundReactionForce(const pinocchio::Force &exp_wrench,
                                const std::vector<Eigen::Vector3d> foot_pos,
                                std::vector<Eigen::Vector3d> &grf);
    void controllLoop();
};