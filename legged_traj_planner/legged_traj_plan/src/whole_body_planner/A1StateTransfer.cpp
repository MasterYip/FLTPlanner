/**
 * @file A1StateTransfer.cpp
 * @author GitHub Copilot
 * @brief A1 State Transfer Implementation for Unitree A1 Robot
 * @version 0.1
 * @date 2025-06-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "legged_traj_plan/whole_body_planner/A1StateTransfer.h"
#include "legged_traj_plan/utils/Geometry.h"

PosList A1FeetPos2PosList(legged_traj_plan::A1FeetPosition feet_pos)
{
    PosList pos_list;
    for (int i = 0; i < 4; ++i) // 4 legs for A1
    {
        pos_list.push_back(Eigen::Vector3d(feet_pos.foot[i].x, feet_pos.foot[i].y, feet_pos.foot[i].z));
    }
    return pos_list;
}

A1StateTransfer::A1StateTransfer(A1_State state0, A1_State state1,
                                 std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner,
                                 bool use_cfg_space) : swing_traj_planner_(swing_traj_planner),
                                                       state0_(state0),
                                                       state1_(state1),
                                                       footpos_list0_(A1FeetPos2PosList(state0.feetPositionNow)),
                                                       footpos_list1_(A1FeetPos2PosList(state1.feetPositionNow)),
                                                       swingtraj_isopt_(std::vector<bool>(4, false)), // 4 legs
                                                       swingtraj_isneeded_(std::vector<bool>(4, false)), // 4 legs
                                                       use_cfg_space_(use_cfg_space)
{
    for (int i = 0; i < 4; ++i) // 4 legs for A1
    {
        swingtraj_isneeded_[i] = (state1.support_State_Now[i] == 0);
    }
}

pinocchio::SE3 A1StateTransfer::eval_torso_traj(double t)
{
    // Convert geometry_msgs::Pose to pinocchio::SE3
    auto pose0 = XYZRPY2SE3(state0_.base_Pose_Now);
    auto pose1 = XYZRPY2SE3(state1_.base_Pose_Now);
    
    pinocchio::Motion err = pinocchio::log6(pose0.actInv(pose1));
    pinocchio::SE3 odom_interp = pose0.act(pinocchio::exp6(err * t));

    return odom_interp;
}

PosList A1StateTransfer::eval_foot_traj(double t, uint derivative, bool auto_opt)
{
    PosList footend_interp;
    for (int i = 0; i < 4; ++i) // 4 legs for A1
    {
        if (swingtraj_isneeded_[i])
        {
            if (!swingtraj_isopt_[i] && auto_opt)
                opt_swing_traj(i);
            if (!use_cfg_space_)
                footend_interp.emplace_back(swingtraj_[i]->evaluate(t, derivative, true));
            else
            {
                Eigen::Vector3d base_pt = swing_traj_planner_->getRobotInterface()->FK_foot(swingtraj_[i]->evaluate(t, derivative, true), i);
                footend_interp.emplace_back(point_SE3Act(eval_torso_traj(t).inverse(), base_pt));
            }
        }
        else
        {
            // Linear interpolation for stance legs
            if (derivative == 0)
                footend_interp.emplace_back(footpos_list0_[i] * (1 - t) + footpos_list1_[i] * t);
            else
                footend_interp.emplace_back(Eigen::Vector3d::Zero());
        }
    }
    return footend_interp;
}

PosList A1StateTransfer::eval_cfg_traj(double t, uint derivative, bool auto_opt)
{
    PosList cfg_interp;
    for (int i = 0; i < 4; ++i) // 4 legs for A1
    {
        if (swingtraj_isneeded_[i])
        {
            if (!swingtraj_isopt_[i] && auto_opt)
                opt_swing_traj(i);

            if (!use_cfg_space_)
                cfg_interp.emplace_back(swing_traj_planner_->getRobotInterface()->IKFast_foot(
                    point_SE3Act(eval_torso_traj(t), swingtraj_[i]->evaluate(t, derivative, true)), i));
            else
                cfg_interp.emplace_back(swingtraj_[i]->evaluate(t, derivative, true));
        }
        else
        {
            // Linear interpolation for stance legs
            if (derivative == 0)
                cfg_interp.emplace_back(swing_traj_planner_->getRobotInterface()->IKFast_foot(
                    point_SE3Act(eval_torso_traj(t), footpos_list0_[i] * (1 - t) + footpos_list1_[i] * t), i));
            else
                cfg_interp.emplace_back(Eigen::Vector3d(nan(""), nan(""), nan("")));
        }
    }
    return cfg_interp;
}

std::array<bool, 4> A1StateTransfer::eval_support_state(double t, double lift_margin, double touch_margin)
{
    std::array<bool, 4> support_state;
    if (t < 1 - touch_margin && t > lift_margin)
    {
        for (int i = 0; i < 4; ++i) // 4 legs for A1
        {
            support_state[i] = (state1_.support_State_Now[i] == 1);
        }
    }
    else
    {
        // All legs in contact during transition
        support_state.fill(true);
    }
    return support_state;
}

void A1StateTransfer::opt_swing_traj(int index)
{
    if (!opt_check(index))
    {
        auto pose0 = XYZRPY2SE3(state0_.base_Pose_Now);
        auto pose1 = XYZRPY2SE3(state1_.base_Pose_Now);

        // Init Trajectory
        swingtraj_[index] = swing_traj_planner_->getInitTraj(
            pose0, pose1, footpos_list0_[index], footpos_list1_[index], index);

        // Optimize Trajectory
        swingtraj_isopt_[index] = swing_traj_planner_->optTraj(
            swingtraj_[index], pose0, pose1, index);

        // Retry Optimization
        if (swing_traj_planner_->getConfig().reOptimize)
        {
            swing_traj_planner_->getConfig().enableLiftRandomize = true;
            bool enableVis = swing_traj_planner_->getConfig().enableVis;
            int reOptCnt = 0;
            while (!opt_check(index) &&
                   reOptCnt < swing_traj_planner_->getConfig().reOptimizeMaxTry)
            {
                reOptCnt++;
                swingtraj_[index] = swing_traj_planner_->getInitTraj(
                    pose0, pose1, footpos_list0_[index], footpos_list1_[index], index);
                swingtraj_isopt_[index] = swing_traj_planner_->optTraj(
                    swingtraj_[index], pose0, pose1, index);
            }
            swing_traj_planner_->getConfig().enableLiftRandomize = false;
            swing_traj_planner_->getConfig().enableVis = enableVis;
        }
        else
        {
            swingtraj_isopt_[index] = true;
        }
    }
}

std::vector<Eigen::Vector3d> A1StateTransfer::generate_footholds(int index, int size, double interval)
{
    auto pose0 = XYZRPY2SE3(state0_.base_Pose_Now);
    auto pose1 = XYZRPY2SE3(state1_.base_Pose_Now);
    auto nominal_foothold = swing_traj_planner_->getRobotInterface()->getNominalFoothold(index);
    nominal_foothold = point_SE3Act(pose1.inverse(), nominal_foothold);
    std::vector<Point3D> footholds;
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            Point3D foothold = nominal_foothold +
                               Point3D((i - size / 2) * interval, (j - size / 2) * interval, 0);
            foothold[2] = swing_traj_planner_->getGridMapInterface()->value(foothold.head(2));
            footholds.emplace_back(foothold);
        }
    }
    return footholds;
}

void A1StateTransfer::reachable_check(int index, int size, double interval)
{
    auto footholds = generate_footholds(index, size, interval);
    std::vector<bool> reachable;
    auto pose0 = XYZRPY2SE3(state0_.base_Pose_Now);
    auto pose1 = XYZRPY2SE3(state1_.base_Pose_Now);
    swing_traj_planner_->reachableCheck(pose0, pose1, footpos_list0_[index], index, footholds, reachable);
}

bool A1StateTransfer::opt_check(int index)
{
    if (index != -1)
    {
        return swingtraj_isopt_[index] || !swingtraj_isneeded_[index];
    }
    else
    {
        for (int i = 0; i < 4; ++i) // 4 legs for A1
        {
            if (!(swingtraj_isopt_[i] || !swingtraj_isneeded_[i]))
            {
                return false;
            }
        }
        return true;
    }
}