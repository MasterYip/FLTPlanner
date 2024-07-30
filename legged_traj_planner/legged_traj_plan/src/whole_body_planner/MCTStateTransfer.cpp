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
                                   std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner,
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

    // Init swing trajectory
    for (int i = 0; i < 6; ++i)
    {
        if (swingtraj_isneeded_[i])
        {
            swingtraj_[i] = swing_traj_planner_->getInitTraj(
                XYZRPY2SE3(state0_.base_Pose_Now), XYZRPY2SE3(state1_.base_Pose_Now),
                footpos_list0_[i], footpos_list1_[i], i);
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

PosList MCTStateTransfer::eval_foot_traj(double t, uint derivative, bool auto_opt)
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
                footend_interp.emplace_back(swingtraj_[i]->evaluate(t, derivative, true));
            }
            else
            {
                Eigen::Vector3d base_pt = swing_traj_planner_->getRobotInterface()->FK_foot(swingtraj_[i]->evaluate(t, derivative, true), i);
                footend_interp.emplace_back(point_SE3Act(eval_torso_traj(t).inverse(), base_pt));
            }
        }
        else
        {
            // Linear interpolation
            if (derivative == 0)
                footend_interp.emplace_back(footpos_list0_[i] * (1 - t) + footpos_list1_[i] * t);
            else
                // FIXME: derivative>0 not implemented
                footend_interp.emplace_back(Eigen::Vector3d::Zero());
        }
    }
    return footend_interp;
}

PosList MCTStateTransfer::eval_cfg_traj(double t, uint derivative, bool auto_opt)
{
    PosList cfg_interp;
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
                cfg_interp.emplace_back(swing_traj_planner_->getRobotInterface()->IKFast_foot(
                    point_SE3Act(eval_torso_traj(t), swingtraj_[i]->evaluate(t, derivative, true)), i));
            }
            else
            {
                cfg_interp.emplace_back(swingtraj_[i]->evaluate(t, derivative, true));
            }
        }
        else
        {
            // Linear interpolation
            // if (derivative == 0)
            //     cfg_interp.push_back(swing_traj_planner_->getRobotInterface()->IKFast_foot(point_SE3Act(eval_torso_traj(0), footpos_list0_[i]), i) * (1 - t) +
            //                          swing_traj_planner_->getRobotInterface()->IKFast_foot(point_SE3Act(eval_torso_traj(1), footpos_list1_[i]), i) * t);
            // else
            //     cfg_interp.push_back(Eigen::Vector3d::Zero());
            if (derivative == 0)
                cfg_interp.emplace_back(swing_traj_planner_->getRobotInterface()->IKFast_foot(
                    point_SE3Act(eval_torso_traj(t), footpos_list0_[i] * (1 - t) + footpos_list1_[i] * t), i));
            else
            {
                // Eigen::Vector3d q = swing_traj_planner_->getRobotInterface()->IKFast_foot(
                //     point_SE3Act(eval_torso_traj(t), footpos_list0_[i] * (1 - t) + footpos_list1_[i] * t), i);
                //     Eigen::Vector3d v_base = (footpos_list1_[i] - footpos_list0_[i]) / (1 - t);
                // Eigen::Matrix3Xd J = swing_traj_planner_->getRobotInterface()->getJacobian(q, i);
                // Eigen::Matrix3Xd Jinv = J.transpose() * (J * J.transpose()).inverse();
                // footend_interp.push_back(Jinv * );

                cfg_interp.emplace_back(Eigen::Vector3d(nan(""), nan(""), nan("")));
            }
            // FIXME: derivative>0 not implemented
            // BUG: vel depends on t
        }
    }
    return cfg_interp;
}

std::array<bool, 6> MCTStateTransfer::eval_support_state(double t, double lift_margin, double touch_margin)
{
    std::array<bool, 6> support_state;
    if (t < 1 - touch_margin && t > lift_margin)
    {
        for (int i = 0; i < 6; ++i)
        {
            support_state[i] = (state1_.support_State_Now[i] == 1);
        }
    }
    else
    {
        // FIXME: Should consider error leg
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

        swingtraj_isopt_[index] = swing_traj_planner_->optTraj(
            swingtraj_[index], pose0, pose1, index);

        // Normal randomization for replanning
        swing_traj_planner_->getConfig().enableLiftRandomize = true;
        swing_traj_planner_->getConfig().enableVis = false;
        while (!opt_check(index))
        {
            swingtraj_[index] = swing_traj_planner_->getInitTraj(
                XYZRPY2SE3(state0_.base_Pose_Now), XYZRPY2SE3(state1_.base_Pose_Now),
                footpos_list0_[index], footpos_list1_[index], index);
            swingtraj_isopt_[index] = swing_traj_planner_->optTraj(
                swingtraj_[index], pose0, pose1, index);
        }
        swing_traj_planner_->getConfig().enableLiftRandomize = false;
        swing_traj_planner_->getConfig().enableVis = true;
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
