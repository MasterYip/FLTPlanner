#include "legged_traj_plan/whole_body_planner/WholeBodyPlanner.h"

// MCTSWholeBodyPlanner::MCTSWholeBodyPlanner()
// {
//     swing_traj_planner = SwingTrajPlanner();
// }

MCTSWholeBodyPlanner::MCTSWholeBodyPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                                           std::shared_ptr<GridMapInterface> gridmap_interface,
                                           std::shared_ptr<ElSpiderAirInterface> robot_interface)
    : gridmap_interface_(gridmap_interface), robot_interface_(robot_interface),
      swing_traj_planner_(std::make_shared<SwingTrajPlanner>(swing_traj_planner_config, robot_interface_, gridmap_interface_)),
      use_cfg_space_(swing_traj_planner_config.useCfgSpace)
{
}

bool MCTSWholeBodyPlanner::enqueue_MCTsolution(hexapod_State state0, hexapod_State state1)
{
    state_trajs.emplace_back(MCTStateTransfer(state0, state1, swing_traj_planner_, use_cfg_space_));
    return true;
}

MCTStateTransfer MCTSWholeBodyPlanner::dequeue_MCTsolution()
{
    MCTStateTransfer ret = state_trajs.front();
    assert(!state_trajs.empty());
    state_trajs.erase(state_trajs.begin());
    return ret;
}

MCTStateTransfer MCTSWholeBodyPlanner::get_state_traj(int index)
{
    return state_trajs.at(index);
}

int MCTSWholeBodyPlanner::get_state_traj_length()
{
    return state_trajs.size();
}

// std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> MCTSWholeBodyPlanner::get_foot_traj(double t, int point_num, double delta) {
//     std::vector<std::vector<double>> default_traj_list(6, std::vector<double>());
//     std::vector<std::vector<double>> opt_traj_list(6, std::vector<double>());
//     int index = 0;
//     while (point_num > 0 && state_trajs.is_valid(index)) {
//         MCTStateTransfer state_traj = state_trajs.at(index);
//         while (t < 1 && point_num > 0) {
//             std::vector<std::vector<double>> foot_pos_list = state_traj.eval_foot_traj(t, false);
//             for (int j = 0; j < 6; ++j) {
//                 // FIXME: It is not recommended to use private var
//                 if (state_traj.opt_check(j) && state_traj.swingtraj_isneeded[j]) {
//                     opt_traj_list[j].insert(opt_traj_list[j].end(), foot_pos_list[j].begin(), foot_pos_list[j].end());
//                 } else {
//                     default_traj_list[j].insert(default_traj_list[j].end(), foot_pos_list[j].begin(), foot_pos_list[j].end());
//                 }
//             }
//             point_num--;
//             t += delta;
//         }
//         index++;
//         t = 0;
//     }
//     return std::make_pair(default_traj_list, opt_traj_list);
// }

RaibertHeuristicPlanner::RaibertHeuristicPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                                                 std::shared_ptr<GridMapInterface> gridmap_interface,
                                                 std::shared_ptr<ElSpiderAirInterface> robot_interface)
    : gridmap_interface_(gridmap_interface), robot_interface_(robot_interface),
      swing_traj_planner_(std::make_shared<SwingTrajPlanner>(swing_traj_planner_config, robot_interface_, gridmap_interface_)),
      use_cfg_space_(swing_traj_planner_config.useCfgSpace)
{
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0));
    switch_scheduler_.emplace_back(LegSwitchScheduler(interval_, duty_, 0.5));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.354, -0.28, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.054, -0.34, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.354, -0.28, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.354, 0.28, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(0.054, 0.34, -0.28));
    nominal_foothold_base_.emplace_back(Eigen::Vector3d(-0.354, 0.28, -0.28));
}

void RaibertHeuristicPlanner::start(pinocchio::SE3 pose, PosList foot_pos_list)
{

    // FIXME: temp
    for (auto &leg_sch : switch_scheduler_)
    {
        leg_sch.reset(ros::Time::now().toSec());
    }
    cmd_vel_extraplator_.update(pose);
    update_time_ = ros::Time::now().toSec();
    for (int i = 0; i < 6; ++i)
    {
        leg_traj_set_[i].setInit(foot_pos_list[i]);
    }
}

void RaibertHeuristicPlanner::update(pinocchio::SE3 pose, geometry_msgs::Twist cmd_vel)
{
    cmd_vel_extraplator_.update(pose, cmd_vel);
    update_time_ = ros::Time::now().toSec();
    preCompute();
}

void RaibertHeuristicPlanner::preCompute(uint swing_traj_num = 1)
{

    double t_lift1, t_down1, t_lift2, t_down2;
    pinocchio::SE3 pose_lift1, pose_mid1, pose_down1, pose_lift2, pose_mid2, pose_down2;
    switch_scheduler_[0].getSucceedingSwitchTimePair(ros::Time::now().toSec(), t_lift1, t_down1);
    switch_scheduler_[1].getSucceedingSwitchTimePair(ros::Time::now().toSec(), t_lift2, t_down2);
    // pose_lift1 = cmd_vel_extraplator_.extrapolate(t_lift1 - update_time_);
    // pose_down1 = cmd_vel_extraplator_.extrapolate(t_down1 - update_time_);
    // pose_lift2 = cmd_vel_extraplator_.extrapolate(t_lift2 - update_time_);
    // pose_down2 = cmd_vel_extraplator_.extrapolate(t_down2 - update_time_);
    pose_mid1 = cmd_vel_extraplator_.extrapolate((t_lift1 + t_down1) / 2 - update_time_);
    pose_mid2 = cmd_vel_extraplator_.extrapolate((t_lift2 + t_down2) / 2 - update_time_);

    for (uint i = 0; i < 6; i += 2)
    {
        Eigen::Vector3d p0 = leg_traj_set_[i].getLastFoothold();
        Eigen::Vector3d p1 = point_SE3Act(pose_mid1, nominal_foothold_base_[i]);
        leg_traj_set_[i].appendStepTraj(p1, swing_traj_planner_->getDefaultTraj(p0, p1, 0.2, 0.1));
    }
    for (uint i = 1; i < 6; i += 2)
    {
        Eigen::Vector3d p0 = leg_traj_set_[i].getLastFoothold();
        Eigen::Vector3d p1 = point_SE3Act(pose_mid2, nominal_foothold_base_[i]);
        leg_traj_set_[i].appendStepTraj(p1, swing_traj_planner_->getDefaultTraj(p0, p1, 0.2, 0.1));
    }

    // for (int i = 0; i < 6; ++i)
    // {
    //     // swingtraj_[i].emplace_back(swing_traj_planner_->getCfgInitTraj(
    // }
}

bool RaibertHeuristicPlanner::query(double t, PosList &foot_pos_list)
{
    pose = cmd_vel_extraplator_.extrapolate(t);
    for (int i = 0; i < 6; ++i)
    {
        foot_pos_list[i] = swing_traj_planner_->query(pose, i);
    }
    return true;
}