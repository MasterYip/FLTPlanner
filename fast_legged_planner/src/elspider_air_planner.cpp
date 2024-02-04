/**
 * @file elspider_air_planner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "fast_legged_planner/robot_interface/ElSpiderAirInterfaceROS.h"
#include "fast_legged_planner/swing_leg_planner/SwingTrajPlanner.h"
#include "fast_legged_planner/perception_interface/GridMapInterface.h"
#include "fast_legged_planner/whole_body_planner/WholeBodyPlanner.h"
#include "fast_legged_planner/hexapod_State.h"
#include <ros/ros.h>
// #include "swing_leg_planner/cost/cost.hpp"
// #include "swing_leg_planner/traj_opt/traj_opt.hpp"

class ElSpiderAirPlanner
{
private:
    ros::NodeHandle nh_;
    ElSpiderAirInterfaceROS robot_interface_;
    GridMapInterface gridmap_interface_;
    HITSpiderWholeBodyPlanner whole_body_planner_;
    std::vector<hexapod_State> MCT_solution_;
    ros::Rate rate_;

public:
    ElSpiderAirPlanner() : robot_interface_(nh_.param("robot_description", std::string(""))), 
                           gridmap_interface_("/grid_map"), whole_body_planner_(gridmap_interface_, robot_interface_),
                           rate_(20)
    {
        ros::Subscriber sub = nh_.subscribe("supportStateTopic", 1, &ElSpiderAirPlanner::callback, this);
    }

    void callback(const fast_legged_planner::hexapod_State &msg)
    {
        MCT_solution_.push_back(msg);
        if (msg.remarks.data == "end_flag")
        {
            ROS_INFO("end_flag received, start planning");
            for (size_t i = 0; i < MCT_solution_.size() - 1; ++i)
            {
                hexapod_State state_0 = MCT_solution_[i];
                hexapod_State state_1 = MCT_solution_[i + 1];
                whole_body_planner_.enqueue_MCTsolution(state_0, state_1);
            }
            MCT_solution_.clear();
            traj_planner();
        }
    }

    void traj_planner()
    {
        double t = 0.0;
        double delta = 0.05;
        while (whole_body_planner_.get_state_traj_length() > 0)
        {
            MCTStateTransfer state_traj = whole_body_planner_.get_state_traj(0);
            pinocchio::SE3 odom_interp = state_traj.eval_torso_traj(t);
            std::vector<Eigen::Vector3d> footend_interp = state_traj.eval_foot_traj(t);
            for (size_t k = 0; k < 6; ++k)
            {
                footend_interp[k] = point_SE3Act(odom_interp, footend_interp[k]);
            }
            robot_interface_.pub_footcmd_from_footendpos(footend_interp);
            robot_interface_.pub_joint_state_from_footendpos(footend_interp);
            robot_interface_.pub_odom(odom_interp);
            t += delta;
            if (t > 1.0)
            {
                t = 0.0;
                whole_body_planner_.dequeue_MCTsolution();
            }
            rate_.sleep();
        }
    }

    void run()
    {
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elspider_air_planner");
    ElSpiderAirPlanner planner;
    planner.run();
    return 0;
}
