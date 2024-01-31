#ifndef MCT_STATE_TRANSFER_H
#define MCT_STATE_TRANSFER_H

#include "pinocchio/algorithm/rpy.hpp"
#include "fast_legged_planner/hexapod_State.h"
#include "fast_legged_planner/utils/Geometry.h"
#include "Eigen/Dense"
// #include "SwingTrajPlanner.h"
#include <vector>

class MCTStateTransfer {
private:
    hexapod_State state0;
    hexapod_State state1;
    SwingTrajPlanner swing_traj_planner;
    pinocchio::SE3* torso_traj;
    std::vector<Eigen::Vector3d> footpos_list0;
    std::vector<Eigen::Vector3d> footpos_list1;
    std::vector<SwingTrajPlanner> swingtraj;
    std::vector<bool> swingtraj_isopt;
    std::vector<bool> swingtraj_isneeded;

public:
    MCTStateTransfer(hexapod_State state0, hexapod_State state1, SwingTrajPlanner swing_traj_planner);
    pinocchio::SE3 eval_torso_traj(double t);
    std::vector<FeetPos2PosList> eval_foot_traj(double t, bool auto_opt=true);
    void opt_swing_traj(int index);
    bool opt_check(int index);
};

#endif // MCT_STATE_TRANSFER_H
