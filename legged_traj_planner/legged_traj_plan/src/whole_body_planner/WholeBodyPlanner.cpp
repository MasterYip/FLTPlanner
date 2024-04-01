#include "legged_traj_plan/whole_body_planner/WholeBodyPlanner.h"

// MCTSWholeBodyPlanner::MCTSWholeBodyPlanner()
// {
//     swing_traj_planner = SwingTrajPlanner();
// }

MCTSWholeBodyPlanner::MCTSWholeBodyPlanner(GridMapInterface &gridmap_interface, BaseRobotInterface &robot_interface)
    : gridmap_interface_(gridmap_interface), robot_interface_(robot_interface),
      swing_traj_planner_(std::make_shared<SwingTrajPlanner>(robot_interface_, gridmap_interface_))
{
}

bool MCTSWholeBodyPlanner::enqueue_MCTsolution(hexapod_State state0, hexapod_State state1)
{
    state_trajs.emplace_back(MCTStateTransfer(state0, state1, swing_traj_planner_));
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
