#include "fast_legged_planner/whole_body_planner/MCTStateTransfer.h"
#include "fast_legged_planner/utils/geometry.h"

MCTStateTransfer::MCTStateTransfer(hexapod_State state0, hexapod_State state1, SwingTrajPlanner swing_traj_planner) {
    this->state0 = state0;
    this->state1 = state1;
    this->swing_traj_planner = swing_traj_planner;
    this->torso_traj = nullptr;
    this->footpos_list0 = FeetPos2PosList(state0.feetPositionNow);
    this->footpos_list1 = FeetPos2PosList(state1.feetPositionNow);
    this->swingtraj = std::vector<SwingTrajPlanner>(6, SwingTrajPlanner());
    this->swingtraj_isopt = std::vector<bool>(6, false);
    this->swingtraj_isneeded = std::vector<bool>(6, false);

    // Default swing trajectory
    double v_lift = 0.1;
    double h_lift = 0.1;
    for (int i = 0; i < 6; ++i) {
        if (this->swingtraj_isneeded[i]) {
            this->swingtraj[i] = this->swing_traj_planner.get_default_traj(
                this->footpos_list0[i], this->footpos_list1[i], v_lift, h_lift);
        }
    }
}

pinocchio::SE3 MCTStateTransfer::eval_torso_traj(double t) {
    // Evaluate torso trajectory at time t
    pinocchio::SE3 pose0 = XYZRPY2SE3(this->state0.base_Pose_Now);
    pinocchio::SE3 pose1 = XYZRPY2SE3(this->state1.base_Pose_Now);
    pinocchio::SE3 err = pinocchio::log(pose0.actInv(pose1));
    pinocchio::SE3 odom_interp = pose0.act(pinocchio::exp(err * t));
    return odom_interp;
}

std::vector<FeetPos2PosList> MCTStateTransfer::eval_foot_traj(double t, bool auto_opt) {
    std::vector<FeetPos2PosList> footend_interp;
    for (int i = 0; i < 6; ++i) {
        if (this->swingtraj_isneeded[i]) {
            if (!this->swingtraj_isopt[i] && auto_opt) {
                this->opt_swing_traj(i);
            }
            footend_interp.push_back(this->swingtraj[i].evaluate(t, true));
        } else {
            footend_interp.push_back(this->footpos_list0[i]);
        }
    }
    return footend_interp;
}

void MCTStateTransfer::opt_swing_traj(int index) {
    if (!this->opt_check(index)) {
        this->swingtraj[index] = this->swing_traj_planner.opt_traj(
            this->swingtraj[index], this->eval_torso_traj, index);
        this->swingtraj_isopt[index] = true;
    }
}

bool MCTStateTransfer::opt_check(int index) {
    if (index != -1) {
        return this->swingtraj_isopt[index] || !this->swingtraj_isneeded[index];
    } else {
        for (int i = 0; i < 6; ++i) {
            if (!(this->swingtraj_isopt[i] || !this->swingtraj_isneeded[i])) {
                return false;
            }
        }
        return true;
    }
}
