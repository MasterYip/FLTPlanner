/**
 * @file MCTStateTransfer.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan/whole_body_planner/MCTStateTransfer.h"
#include "legged_traj_plan/utils/Geometry.h"

PosList FeetPos2PosList(legged_traj_plan::FeetPosition feet_pos)
{
    PosList pos_list;
    for (int i = 0; i < 6; ++i)
    {
        pos_list.push_back(Eigen::Vector3d(feet_pos.foot[i].x, feet_pos.foot[i].y, feet_pos.foot[i].z));
    }
    return pos_list;
}

MCTStateTransfer::MCTStateTransfer(hexapod_State state0, hexapod_State state1,
                                   std::shared_ptr<SwingTrajPlanner> swing_traj_planner,
                                   bool use_cfg_space) : swing_traj_planner_(swing_traj_planner),
                                                         state0_(state0),
                                                         state1_(state1),
                                                         footpos_list0_(FeetPos2PosList(state0.feetPositionNow)),
                                                         footpos_list1_(FeetPos2PosList(state1.feetPositionNow)),
                                                         swingtraj_isopt_(std::vector<bool>(6, false)),
                                                         swingtraj_isneeded_(std::vector<bool>(6, false)),
                                                         use_cfg_space_(use_cfg_space)
{
    for (int i = 0; i < 6; ++i)
    {
        swingtraj_isneeded_[i] = (state1.support_State_Now[i] == 0);
    }

    // Default swing trajectory
    double v_lift = 0.2; // NOTE: not used
    double h_lift = 0.10;
    for (int i = 0; i < 6; ++i)
    {
        if (swingtraj_isneeded_[i])
        {
            if (!use_cfg_space_)
            {
                // Default Swing Trajectory
                // swingtraj_[i] = swing_traj_planner_->getDefaultTraj(
                //     footpos_list0_[i], footpos_list1_[i], v_lift, h_lift);
                // GCS Search Traj
                swingtraj_[i] = swing_traj_planner_->getInitTraj(
                    XYZRPY2SE3(state0_.base_Pose_Now), XYZRPY2SE3(state1_.base_Pose_Now),
                    footpos_list0_[i], footpos_list1_[i], v_lift, h_lift, i);
            }
            else
            {
                swingtraj_[i] = swing_traj_planner_->getCfgInitTraj(
                    XYZRPY2SE3(state0_.base_Pose_Now), XYZRPY2SE3(state1_.base_Pose_Now),
                    footpos_list0_[i], footpos_list1_[i], v_lift, i);
            }
        }
    }
}

pinocchio::SE3 MCTStateTransfer::eval_torso_traj(double t)
{
    // Evaluate torso trajectory at time t
    pinocchio::SE3 pose0 = XYZRPY2SE3(state0_.base_Pose_Now);
    pinocchio::SE3 pose1 = XYZRPY2SE3(state1_.base_Pose_Now);
    pinocchio::Motion err = pinocchio::log6(pose0.actInv(pose1));
    pinocchio::SE3 odom_interp = pose0.act(pinocchio::exp6(err * t));

    return odom_interp;
}

PosList MCTStateTransfer::eval_foot_traj(double t, bool auto_opt)
{
    PosList footend_interp;
    for (int i = 0; i < 6; ++i)
    {
        if (swingtraj_isneeded_[i])
        {
            if (!swingtraj_isopt_[i] && auto_opt)
            {
                opt_swing_traj(i);
            }
            if (!use_cfg_space_)
            {
                footend_interp.push_back(swingtraj_[i]->evaluate(t, 0, true));
            }
            else
            {
                Eigen::Vector3d base_pt = swing_traj_planner_->getRobotInterface().FK_foot(swingtraj_[i]->evaluate(t, 0, true), i);
                footend_interp.push_back(point_SE3Act(eval_torso_traj(t).inverse(), base_pt));
            }
        }
        else
        {
            // Linear interpolation
            footend_interp.push_back(footpos_list0_[i] * (1 - t) + footpos_list1_[i] * t);
        }
    }
    return footend_interp;
}

std::array<bool, 6> MCTStateTransfer::eval_support_state(double t, double margin)
{
    std::array<bool, 6> support_state;
    if (t < 1 - margin && t > margin)
    {
        for (int i = 0; i < 6; ++i)
        {
            support_state[i] = (state1_.support_State_Now[i] == 1);
        }
    }
    else
    {
        support_state.fill(true);
    }
    return support_state;
}

void MCTStateTransfer::opt_swing_traj(int index)
{
    if (!opt_check(index))
    {
        pinocchio::SE3 pose0 = XYZRPY2SE3(state0_.base_Pose_Now);
        pinocchio::SE3 pose1 = XYZRPY2SE3(state1_.base_Pose_Now);
        if (use_cfg_space_)
        {
            swingtraj_isopt_[index] = swing_traj_planner_->optCfgTraj(
                swingtraj_[index], pose0, pose1, index);
        }
        else
        {
            swingtraj_isopt_[index] = swing_traj_planner_->optTraj(
                swingtraj_[index], pose0, pose1, index);
        }
    }
}

/**
 * @brief Check if the traj does not need to be optimized
 *
 * @param index foot index (-1 for all)
 * @return true: does not need to be optimized
 * @return false: need to be optimized
 */
bool MCTStateTransfer::opt_check(int index = -1)
{
    if (index != -1)
    {
        return swingtraj_isopt_[index] || !swingtraj_isneeded_[index];
    }
    else
    {
        for (int i = 0; i < 6; ++i)
        {
            if (!(swingtraj_isopt_[i] || !swingtraj_isneeded_[i]))
            {
                return false;
            }
        }
        return true;
    }
}
