/**
 * @file WholeBodyPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <geometry_msgs/Twist.h>
/* internal project header files */

#include "legged_traj_plan/utils/CircleQueue.h"
#include "legged_traj_plan/swing_leg_planner/SwingTrajPlanner.h"
#include "legged_traj_plan/whole_body_planner/MCTStateTransfer.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
class MCTSWholeBodyPlanner
{
private:
    std::vector<MCTStateTransfer> state_trajs;
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    std::shared_ptr<SwingTrajPlanner> swing_traj_planner_;
    bool use_cfg_space_;

public:
    MCTSWholeBodyPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                         std::shared_ptr<GridMapInterface> gridmap_interface,
                         std::shared_ptr<ElSpiderAirInterface> robot_interface);

    bool enqueue_MCTsolution(hexapod_State state0, hexapod_State state1);

    MCTStateTransfer dequeue_MCTsolution();

    MCTStateTransfer get_state_traj(int index);

    int get_state_traj_length();

    // std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> get_foot_traj(double t, int point_num, double delta);
};

class LegSwitchScheduler
{
private:
    double interval_;
    double duty_;
    double phase_shift_;

    double start_time_ = 0;
    bool is_running_ = false;

public:
    // FIXME: add phase
    /**
     * @brief Construct a new Leg Switch Scheduler object
     *
     * @param interval Interval of the scheduler (sec)
     * @param duty Duty (0~1)
     * @param phase_shift_ Phase shift (0~1) - apply DELAY to the scheduler
     */
    LegSwitchScheduler(double interval, double duty, double phase_shift = 0)
        : interval_(interval), duty_(duty), phase_shift_(phase_shift){};

    /**
     * @brief Reset scheduler
     * @note [___stance___```swing```|___stance___```swing```]
     * @param start_time
     */
    void reset(double start_time)
    {
        start_time_ = start_time;
        is_running_ = true;
    };

    /**
     * @brief Query the support state at time t
     *
     * @param t
     * @return true Stance
     * @return false Swing
     */
    bool querySuppotState(double t)
    {
        if (!is_running_)
            return true;
        double t_local = t - start_time_ - phase_shift_ * interval_;
        if (t_local < 0)
            return true;

        double t_local_mod = fmod(t_local, interval_);
        if (t_local_mod < duty_ * interval_)
            return true;
        else
            return false;
    }

    bool getSucceedingSwitchTimePair(double t, &double t_lift, &double t_touch,
                                     uint succeed_num = 0)
    {
        if (!is_running_)
            return false;
        double t_local = t - start_time_ - phase_shift_ * interval_;
        if (t_local < 0)
            return false;
        double t_local_mod = fmod(t_local, interval_);
        if (t_local_mod < duty_ * interval_)
        {
            t_lift = (t - t_local_mod + duty_ * interval_) + interval_ * succeed_num;
            t_touch = (t - t_local_mod + interval_) + interval_ * succeed_num;
            return true;
        }
        else
        {
            t_lift = (t - t_local_mod + duty_ * interval_ + interval_) + interval_ * succeed_num;
            t_touch = (t - t_local_mod + interval_ + interval_) + interval_ * succeed_num;
            return true;
        }
    }
}

// FIXME: temporarily use workspace traj
class LegTrajSet
{
private:
    std::vector<Eigen::Vector3d> foothold_;
    std::vector<std::shared_ptr<TrajectoryBase>> swing_traj_;

public:
    LegTrajSet() = default;

    setInit(Eigen::Vector3d foothold){
        foothold_.clear();
        foothold_.emplace_back(foothold);
    };

    void appendStepTraj(Eigen::Vector3d foothold, std::shared_ptr<TrajectoryBase> swing_traj)
    {
        swing_traj_.emplace_back(swing_traj);
        foothold_.emplace_back(foothold);
    }

    void popStepTraj()
    {
        swing_traj_.pop_front();
        foothold_.pop_front();
    }

    Eigen::Vector3d getLastFoothold()
    {
        return foothold_.back();
    }
}

class CmdVelExtraplator
{
private:
    geometry_msgs::Twist cmd_vel_; // cmd vel relative to the BASE frame
    pinocchio::SE3 pose_;

public:
    CmdVelExtraplator() = default;
    void update(pinocchio::SE3 pose, geometry_msgs::Twist cmd_vel = geometry_msgs::Twist())
    {
        // FIXME: is default cmd_vel 0?
        cmd_vel_ = cmd_vel;
        pose_ = pose;
    }

    /**
     * @brief Extrapolate the pose by dt
     *
     * @param dt Delta t
     * @return pinocchio::SE3
     */
    pinocchio::SE3 extrapolate(double dt)
    {
        pinocchio::SE3 pose_new = pose_;
        Eigen::Vector3d linear_world = pose_.rotation().transpose() * cmd_vel_.linear;
        Eigen::Vector3d angular_world = pose_.rotation().transpose() * cmd_vel_.angular;
        pose_new.translation() += linear_world * dt;
        pose_new.rotation() = pose_.rotation() * pinocchio::SE3::exp(angular_world * dt).rotation();
        return pose_new;
    }

}

// class RaibertHeuristicSelector
// {
// private:

// }

class RaibertHeuristicPlanner
{
private:
    double interval_ = 1.0;
    double duty_ = 0.5;

    PosList nominal_foothold_base_;
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    std::shared_ptr<SwingTrajPlanner> swing_traj_planner_;
    CmdVelExtraplator cmd_vel_extraplator_;
    std::vector<LegSwitchScheduler> switch_scheduler_;
    std::vector<LegTrajSet> leg_traj_set_(6);
    double update_time_ = 0;
    bool use_cfg_space_;

public:
    RaibertHeuristicPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                            std::shared_ptr<GridMapInterface> gridmap_interface,
                            std::shared_ptr<ElSpiderAirInterface> robot_interface);

    void start(pinocchio::SE3 pose, PosList foot_pos_list);

    void update(pinocchio::SE3 pose, geometry_msgs::Twist cmd_vel);
}