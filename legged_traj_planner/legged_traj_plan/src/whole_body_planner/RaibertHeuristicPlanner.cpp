/**
 * @file RaibertHeuristicPlanner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan/whole_body_planner/RaibertHeuristicPlanner.h"

SimpleRaibertPlanner::SimpleRaibertPlanner(std::shared_ptr<GridMapInterface> gridmap_interface,
                                           std::shared_ptr<BaseRobotInterface> robot_interface)
    : gridmap_interface_(gridmap_interface), robot_interface_(robot_interface)
{
    // Bigait
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
    // Trible-gait
    // switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
    // switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.33));
    // switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.66));
    // switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.66));
    // switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.33));
    // switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));

    nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.354, -0.28 - 0.04, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.054, -0.34 - 0.04, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.354, -0.28 - 0.04, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.354, 0.28 + 0.04, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.054, 0.34 + 0.04, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.354, 0.28 + 0.04, -0.28));

    PosList pose_sample_pts;
    for (double x = -0.4; x <= 0.4; x += 0.2)
    {
        for (double y = -0.4; y <= 0.4; y += 0.2)
        {
            pose_sample_pts.emplace_back(Eigen::Vector3d(x, y, 0));
        }
    }
    cmd_vel_extrapolator_.init(gridmap_interface, pose_sample_pts);

    last_footholds_.resize(6);
    next_footholds_.resize(6);
}

void SimpleRaibertPlanner::start(pinocchio::SE3 pose, PosList foot_pos_list)
{
    for (auto &leg_sch : switch_scheduler_)
    {
        leg_sch.reset(ros::Time::now().toSec());
    }
    if (foot_pos_list.size() != 6)
    {
        foot_pos_list.clear();
        for (int i = 0; i < 6; ++i)
        {
            foot_pos_list.emplace_back(point_SE3Act(pose.inverse(), nominal_foothold_base_[i]));
        }
    }
    update(pose, geometry_msgs::Twist(), foot_pos_list);
}

void SimpleRaibertPlanner::update(const pinocchio::SE3 pose, const geometry_msgs::Twist cmd_vel,
                                  const PosList last_footholds)
{
    update_time_ = ros::Time::now().toSec();
    cmd_vel_extrapolator_.update(pose, cmd_vel);
    pose_ = pose;
    cmd_vel_ = cmd_vel;

    for (uint i = 0; i < 6; ++i)
    {
        if (!switch_scheduler_[i].querySuppotState(update_time_)) // In swing phase
        {
            double t_next_stmid;
            switch_scheduler_[i].getNextStMidTime(update_time_, t_next_stmid);
            pinocchio::SE3 pose_st_mid = cmd_vel_extrapolator_.extrapolate(t_next_stmid - update_time_);
            next_footholds_[i] = point_SE3Act(pose_st_mid.inverse(), nominal_foothold_base_[i]);
            next_footholds_[i].z() = gridmap_interface_->value(grid_map::Position(next_footholds_[i].x(), next_footholds_[i].y()));
        }
        else
        {
            last_footholds_[i] = next_footholds_[i];
        }
    }

    // Reset footholds
    if (last_footholds.size() == 6) // Not empty
    {
        last_footholds_ = last_footholds;
        next_footholds_ = last_footholds;
    }
}

/**
 * @brief
 *
 * @param t
 * @param pose in WORLD frame
 * @param foot_pos_list in WORLD frame
 * @param support_state
 * @return true
 * @return false
 */
bool SimpleRaibertPlanner::query(double t, pinocchio::SE3 &pose,
                                 PosList &foot_pos_list,
                                 std::array<bool, 6> &support_state)
{
    pose = cmd_vel_extrapolator_.extrapolate(t - update_time_);
    foot_pos_list.clear();
    support_state.fill(true);
    for (int i = 0; i < 6; i++)
    {
        double progress;
        if (switch_scheduler_[i].querySwingState(t, progress))
        {
            Eigen::Matrix<double, 4, 3> hermite_knots;
            hermite_knots.row(0) = last_footholds_[i];
            hermite_knots.row(1) << 0, 0, vLift_;
            hermite_knots.row(2) = next_footholds_[i];
            hermite_knots.row(3) << 0, 0, -vLift_;
            foot_pos_list.emplace_back(cubic_evaluate(HERMITE_COE_MAT, hermite_knots, progress, 0));
            support_state[i] = false;
        }
        else
        {
            foot_pos_list.emplace_back(next_footholds_[i]);
        }
    }
    return true;
}

// RaibertHeuristicPlanner

RaibertHeuristicPlanner::RaibertHeuristicPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                                                 std::shared_ptr<GridMapInterface> gridmap_interface,
                                                 std::shared_ptr<BaseRobotInterface> robot_interface)
    : gridmap_interface_(gridmap_interface), robot_interface_(robot_interface),
      swing_traj_planner_(std::make_shared<FLTCfgPlanner>(swing_traj_planner_config, robot_interface_, gridmap_interface_))
{
    // Determine number of legs from robot interface
    num_legs_ = determineNumLegs();

    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "raibert_heuristic_planner_vis");

    // Configure schedulers based on robot type
    if (num_legs_ == 4) // Quadruped - trotting gait
    {
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));   // FR
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5)); // FL
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5)); // RR
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));   // RL

        // Quadruped nominal footholds (A1 style)
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.18, -0.13, -0.32));  // FR
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.18, 0.13, -0.32));   // FL
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.18, -0.13, -0.32)); // RR
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.18, 0.13, -0.32));  // RL
    }
    else // Hexapod - default bigait
    {
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));

        // Hexapod nominal footholds
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.354, -0.28 - 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.054, -0.34 - 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.354, -0.28 - 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.354, 0.28 + 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.054, 0.34 + 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.354, 0.28 + 0.04, -0.28));
    }

    PosList pose_sample_pts;
    for (double x = -0.4; x <= 0.4; x += 0.1)
    {
        for (double y = -0.4; y <= 0.4; y += 0.1)
        {
            pose_sample_pts.emplace_back(Eigen::Vector3d(x, y, 0));
        }
    }
    cmd_vel_extrapolator_.init(gridmap_interface, pose_sample_pts);

    leg_traj_.resize(num_legs_);

    // Initialize reachability check parameters from config
    enable_reachable_check_ = swing_traj_planner_->getConfig().enableVis; // Use vis flag as default
    reachable_check_size_ = 30;
    reachable_check_interval_ = 0.025;
}

RaibertHeuristicPlanner::RaibertHeuristicPlanner(std::shared_ptr<SwingTrajPlannerBase> swing_traj_planner_,
                                                 std::shared_ptr<GridMapInterface> gridmap_interface,
                                                 std::shared_ptr<BaseRobotInterface> robot_interface)
    : gridmap_interface_(gridmap_interface), robot_interface_(robot_interface),
      swing_traj_planner_(swing_traj_planner_)
{
    // Determine number of legs from robot interface
    num_legs_ = determineNumLegs();

    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "raibert_heuristic_planner_vis");

    // Configure schedulers based on robot type
    if (num_legs_ == 4) // Quadruped - trotting gait
    {
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));   // FR
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5)); // FL
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5)); // RR
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));   // RL

        // Quadruped nominal footholds (A1 style)
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.18, -0.13, -0.32));  // FR
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.18, 0.13, -0.32));   // FL
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.18, -0.13, -0.32)); // RR
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.18, 0.13, -0.32));  // RL
    }
    else // Hexapod - default bigait
    {
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
        switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));

        // Hexapod nominal footholds
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.354, -0.28 - 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.054, -0.34 - 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.354, -0.28 - 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.354, 0.28 + 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.054, 0.34 + 0.04, -0.28));
        nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.354, 0.28 + 0.04, -0.28));
    }

    PosList pose_sample_pts;
    for (double x = -0.4; x <= 0.4; x += 0.1)
    {
        for (double y = -0.4; y <= 0.4; y += 0.1)
        {
            pose_sample_pts.emplace_back(Eigen::Vector3d(x, y, 0));
        }
    }
    cmd_vel_extrapolator_.init(gridmap_interface, pose_sample_pts);

    leg_traj_.resize(num_legs_);

    // Initialize reachability check parameters from config
    enable_reachable_check_ = swing_traj_planner_->getConfig().enableVis; // Use vis flag as default
    reachable_check_size_ = 30;
    reachable_check_interval_ = 0.025;
}

void RaibertHeuristicPlanner::start(pinocchio::SE3 pose, PosList foot_pos_list)
{
    for (auto &leg_sch : switch_scheduler_)
    {
        leg_sch.reset(ros::Time::now().toSec());
    }
    update(pose, geometry_msgs::Twist(), foot_pos_list);
}

/**
 * @brief
 *
 * @param pose
 * @param cmd_vel Cmd in BASE frame
 * @param foot_pos_list Foot pos now in WORLD frame
 */
void RaibertHeuristicPlanner::update(pinocchio::SE3 pose, geometry_msgs::Twist cmd_vel,
                                     PosList foot_pos_list)
{
    cmd_vel_extrapolator_.update(pose, cmd_vel);
    update_time_ = ros::Time::now().toSec();

    bool foot_pos_list_valid = foot_pos_list.size() == num_legs_;

    // Vis Clear
    swing_traj_planner_->visClear();

    std::vector<std::pair<double, double>> switch_time_pairs;
    pinocchio::SE3 pose_st_mid, pose_touch, pose_lift;
    double t_now = update_time_;
    for (int i = 0; i < num_legs_; ++i)
    {
        // Remove old traj
        while (leg_traj_[i].size() > 1 && leg_traj_[i].front().t_touch < t_now - interval_)
        {
            leg_traj_[i].erase(leg_traj_[i].begin());
        }

        if (switch_scheduler_[i].getEventTimes(t_now, t_now + extrapolate_window_,
                                               switch_time_pairs))
        {
            // Find index in leg_traj_
            int index = leg_traj_[i].size();
            double t_mid = (switch_time_pairs[0].first + switch_time_pairs[0].second) / 2; // First time pair
            for (int j = 0; j < leg_traj_[i].size(); ++j)
            {
                if (leg_traj_[i][j].isApproxTmid(t_mid))
                {
                    index = j;
                    break;
                }
            }

            for (int j = 0; j < switch_time_pairs.size() - 1; j++)
            {
                auto &pair = switch_time_pairs[j];
                auto &pair_next = switch_time_pairs[j + 1];
                pose_lift = cmd_vel_extrapolator_.extrapolate(pair.first - update_time_);
                pose_st_mid = cmd_vel_extrapolator_.extrapolate((pair_next.first + pair.second) / 2 - update_time_);
                pose_touch = cmd_vel_extrapolator_.extrapolate(pair.second - update_time_);
                Eigen::Vector3d p0;
                Eigen::Vector3d p1 = point_SE3Act(pose_st_mid.inverse(), nominal_foothold_base_[i]);
                p1.z() = gridmap_interface_->value(grid_map::Position(p1.x(), p1.y()));

                if (j == 0 && foot_pos_list_valid)
                {
                    p0 = foot_pos_list[i];
                }
                else if (index > 0)
                {
                    p0 = leg_traj_[i][index - 1].foothold_touch;
                }
                else
                {
                    // FIXME: temp solution
                    ROS_WARN("leg_traj_ is empty or there are no previous touch down footholds");
                    p0 = point_SE3Act(pose_lift.inverse(), nominal_foothold_base_[i]);
                }

                // Perform reachability check for this leg before planning trajectory
                if (enable_reachable_check_ && j == 0 && i == 0) // Only check for the first upcoming swing
                {
                    // Generate grid of potential footholds around the target position
                    auto footholds = generateReachabilityGrid(p1, reachable_check_size_, reachable_check_interval_);

                    // Perform reachability check using the actual lift and touch poses
                    std::vector<bool> reachable;
                    swing_traj_planner_->reachableCheck(pose_lift, pose_touch, p0, i, footholds, reachable);
                }

                if (index < leg_traj_[i].size())
                {
                    // FIXME: temp solution
                    // Eigen::Vector3d lift_pos_cfg, lift_vel_cfg, touch_pos_cfg, touch_vel_cfg;
                    // Eigen::Vector3d lift_normal = gridmap_interface_->sdfDerivative(p0, 0);
                    // Eigen::Vector3d touch_normal = gridmap_interface_->sdfDerivative(p1, 0);
                    // lift_normal.normalize();
                    // touch_normal.normalize();
                    // Eigen::Vector3d v0 = lift_normal * vLift;
                    // Eigen::Vector3d v1 = -touch_normal * vLift;
                    // toCfgSpace(pose, p0, v0, lift_pos_cfg, lift_vel_cfg, i);
                    // toCfgSpace(pose, p1, v1, touch_pos_cfg, touch_vel_cfg, i);
                    // leg_traj_[i].at(index).update(pair.first, pair.second, p0, p1, lift_pos_cfg, touch_pos_cfg, lift_vel_cfg, touch_vel_cfg);

                    // Get Init Traj
                    // leg_traj_[i].at(index) = LegTraj(pair.first, pair.second, p0, p1,
                    //                                  swing_traj_planner_->getInitTraj(pose_lift, pose_touch, p0, p1, i));
                    // Get Opt Traj
                    std::shared_ptr<TrajectoryBase> traj = swing_traj_planner_->getInitTraj(pose_lift, pose_touch, p0, p1, i);
                    swing_traj_planner_->optTraj(traj, pose_lift, pose_touch, i);
                    leg_traj_[i].at(index) = LegTraj(pair.first, pair.second, p0, p1, traj);

                    index++;
                }
                else
                {
                    // leg_traj_[i].emplace_back(LegTraj(pair.first, pair.second, p0, p1,
                    //                                   swing_traj_planner_->getInitTraj(pose_lift, pose_touch, p0, p1, i)));

                    // Get Opt Traj
                    std::shared_ptr<TrajectoryBase> traj = swing_traj_planner_->getInitTraj(pose_lift, pose_touch, p0, p1, i);
                    swing_traj_planner_->optTraj(traj, pose_lift, pose_touch, i);
                    leg_traj_[i].emplace_back(LegTraj(pair.first, pair.second, p0, p1, traj));

                    index++;
                }
            }
        }
    }

    // Visualization
    if (swing_traj_planner_->getConfig().enableVis)
    {
        visualizer_->delAll();
        std::vector<std::vector<Point3D>> foot_trajs(num_legs_);
        double delta_t = 0.01;
        int sample_points = 100;
        pinocchio::SE3 pose;
        PosList foot_pos_list;
        std::vector<bool> support_state;
        for (int i = 0; i < sample_points; i++)
        {
            double t = update_time_ + i * delta_t;
            queryGeneric(t, pose, foot_pos_list, support_state);
            for (int j = 0; j < num_legs_; j++)
            {
                foot_trajs[j].emplace_back(point_SE3Act(pose.inverse(), foot_pos_list[j]));
            }
        }
        for (int i = 0; i < num_legs_; i++)
        {
            visualizer_->visCurve(foot_trajs[i]);
        }
    }
}

/**
 * @brief
 *
 * @param t World time
 * @param pose Expected body pose in WORLD frame
 * @param foot_pos_list Expected foot pos in BASE frame
 * @param support_state Foot support state
 * @return true
 * @return false
 */
bool RaibertHeuristicPlanner::query(double t, pinocchio::SE3 &pose,
                                    PosList &foot_pos_list,
                                    std::array<bool, 6> &support_state)
{
    foot_pos_list.clear();
    support_state.fill(true);
    pose = cmd_vel_extrapolator_.extrapolate(t - update_time_);
    for (int i = 0; i < 6; ++i)
    {
        bool found = false;
        for (int j = 0; j < leg_traj_[i].size(); j++)
        {
            auto &leg_traj = leg_traj_[i][j];
            if (leg_traj.isInDuration(t))
            {
                foot_pos_list.emplace_back(robot_interface_->FK_foot(leg_traj.evaluate(t), i));
                support_state[i] = false;
                found = true;
                break;
            }
            if (t < leg_traj.t_lift)
            {
                foot_pos_list.emplace_back(point_SE3Act(pose, leg_traj.foothold_lift));
                found = true;
                break;
            }
        }
        if (!found)
        {
            if (leg_traj_[i].size() > 0)
                foot_pos_list.emplace_back(point_SE3Act(pose, leg_traj_[i].back().foothold_touch));
            else
            {
                ROS_WARN("No valid leg_traj found for leg %d at time %f", i, t);
                foot_pos_list.emplace_back(nominal_foothold_base_[i]);
            }
        }
    }
    return true;
}

/**
 * @brief
 *
 * @param t World time
 * @param pose Expected body pose in WORLD frame
 * @param foot_pos_list Expected foot pos in Config space
 * @param support_state Foot support state
 * @return true
 * @return false
 */
bool RaibertHeuristicPlanner::queryCfg(double t, pinocchio::SE3 &pose,
                                       PosList &foot_pos_list,
                                       std::array<bool, 6> &support_state)
{
    foot_pos_list.clear();
    support_state.fill(true);
    pose = cmd_vel_extrapolator_.extrapolate(t - update_time_);
    for (int i = 0; i < 6; ++i)
    {
        bool found = false;
        for (int j = 0; j < leg_traj_[i].size(); j++)
        {
            auto &leg_traj = leg_traj_[i][j];
            if (leg_traj.isInDuration(t))
            {
                foot_pos_list.emplace_back(leg_traj.evaluate(t));
                support_state[i] = false;
                found = true;
                break;
            }
            if (t < leg_traj.t_lift)
            {
                foot_pos_list.emplace_back(robot_interface_->IKFast_foot(point_SE3Act(pose, leg_traj.foothold_lift), i));
                found = true;
                break;
            }
        }
        if (!found)
        {
            if (leg_traj_[i].size() > 0)
                foot_pos_list.emplace_back(robot_interface_->IKFast_foot(point_SE3Act(pose, leg_traj_[i].back().foothold_touch), i));
            else
            {
                ROS_WARN("No valid leg_traj found for leg %d at time %f", i, t);
                foot_pos_list.emplace_back(robot_interface_->IKFast_foot(nominal_foothold_base_[i], i));
            }
        }
    }
    return true;
}

bool RaibertHeuristicPlanner::toCfgSpace(pinocchio::SE3 pose, Eigen::Vector3d pos, Eigen::Vector3d vel,
                                         Eigen::Vector3d &pos_cfg, Eigen::Vector3d &vel_cfg,
                                         int leg_index)
{
    Eigen::Vector3d base_pos = point_SE3Act(pose, pos);
    Eigen::Vector3d base_vel = vec_SE3Act(pose, vel);
    pos_cfg = robot_interface_->IKFast_foot(pos, leg_index);
    Eigen::Matrix3Xd J = robot_interface_->getJacobian(pos_cfg, leg_index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    vel_cfg = J_inv * base_vel;
    return true;
}

// Quadruped interface (4 legs) - new overloaded methods
bool RaibertHeuristicPlanner::query(double t, pinocchio::SE3 &pose,
                                    PosList &foot_pos_list,
                                    std::array<bool, 4> &support_state)
{
    // Use the generic interface and convert to quadruped-specific format
    std::vector<bool> generic_support_state;
    bool result = queryGeneric(t, pose, foot_pos_list, generic_support_state);

    // Convert to 4-leg array
    for (int i = 0; i < 4 && i < generic_support_state.size(); ++i)
    {
        support_state[i] = generic_support_state[i];
    }

    return result;
}

bool RaibertHeuristicPlanner::queryCfg(double t, pinocchio::SE3 &pose,
                                       PosList &foot_pos_list,
                                       std::array<bool, 4> &support_state)
{
    // Use the generic interface and convert to quadruped-specific format
    std::vector<bool> generic_support_state;
    bool result = queryCfgGeneric(t, pose, foot_pos_list, generic_support_state);

    // Convert to 4-leg array
    for (int i = 0; i < 4 && i < generic_support_state.size(); ++i)
    {
        support_state[i] = generic_support_state[i];
    }

    return result;
}

// Generic interface that works with both robot types
bool RaibertHeuristicPlanner::queryGeneric(double t, pinocchio::SE3 &pose,
                                           PosList &foot_pos_list,
                                           std::vector<bool> &support_state)
{
    foot_pos_list.clear();
    support_state.clear();
    support_state.resize(num_legs_, true);

    pose = cmd_vel_extrapolator_.extrapolate(t - update_time_);
    for (int i = 0; i < num_legs_; ++i)
    {
        bool found = false;
        for (int j = 0; j < leg_traj_[i].size(); j++)
        {
            auto &leg_traj = leg_traj_[i][j];
            if (leg_traj.isInDuration(t))
            {
                foot_pos_list.emplace_back(robot_interface_->FK_foot(leg_traj.evaluate(t), i));
                support_state[i] = false;
                found = true;
                break;
            }
            if (t < leg_traj.t_lift)
            {
                foot_pos_list.emplace_back(point_SE3Act(pose, leg_traj.foothold_lift));
                found = true;
                break;
            }
        }
        if (!found)
        {
            if (leg_traj_[i].size() > 0)
                foot_pos_list.emplace_back(point_SE3Act(pose, leg_traj_[i].back().foothold_touch));
            else
            {
                ROS_WARN("No valid leg_traj found for leg %d at time %f", i, t);
                foot_pos_list.emplace_back(nominal_foothold_base_[i]);
            }
        }
    }
    return true;
}

bool RaibertHeuristicPlanner::queryCfgGeneric(double t, pinocchio::SE3 &pose,
                                              PosList &foot_pos_list,
                                              std::vector<bool> &support_state)
{
    foot_pos_list.clear();
    support_state.clear();
    support_state.resize(num_legs_, true);

    pose = cmd_vel_extrapolator_.extrapolate(t - update_time_);
    for (int i = 0; i < num_legs_; ++i)
    {
        bool found = false;
        for (int j = 0; j < leg_traj_[i].size(); j++)
        {
            auto &leg_traj = leg_traj_[i][j];
            if (leg_traj.isInDuration(t))
            {
                foot_pos_list.emplace_back(leg_traj.evaluate(t));
                support_state[i] = false;
                found = true;
                break;
            }
            if (t < leg_traj.t_lift)
            {
                foot_pos_list.emplace_back(robot_interface_->IKFast_foot(point_SE3Act(pose, leg_traj.foothold_lift), i));
                found = true;
                break;
            }
        }
        if (!found)
        {
            if (leg_traj_[i].size() > 0)
                foot_pos_list.emplace_back(robot_interface_->IKFast_foot(point_SE3Act(pose, leg_traj_[i].back().foothold_touch), i));
            else
            {
                ROS_WARN("No valid leg_traj found for leg %d at time %f", i, t);
                foot_pos_list.emplace_back(robot_interface_->IKFast_foot(nominal_foothold_base_[i], i));
            }
        }
    }
    return true;
}