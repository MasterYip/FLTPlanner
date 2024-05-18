/**
 * @file RaibertHeuristicPlanner.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/swing_leg_planner/SwingTrajPlanner.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
/* external project header files */

using PosList = std::vector<Eigen::Vector3d>;

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
     * @param duty Duty (0~1) of STANCE phase
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

    /**
     * @brief Query the swing state at time t
     *
     * @param[in] t
     * @param[out] progress Progress of the swing phase
     * @return true In swing phase
     * @return false
     */
    bool querySwingState(double t, double &progress)
    {
        if (!is_running_)
            return false;
        double t_local = t - start_time_ - phase_shift_ * interval_;
        if (t_local < 0)
            return false;

        double t_local_mod = fmod(t_local, interval_);
        if (t_local_mod > duty_ * interval_)
        {
            progress = (t_local_mod - duty_ * interval_) / (interval_ - duty_ * interval_);
            return true;
        }
        else
            return false;
    }

    /**
     * @brief Get the Next Stand Mid Time
     *
     * @param[in] t
     * @param[out] t_mid
     * @return true
     * @return false
     */
    bool getNextStMidTime(double t, double &t_mid)
    {
        if (!is_running_)
            return false;
        double t_local = t - start_time_ - phase_shift_ * interval_;
        if (t_local < 0)
            return false;

        double t_local_mod = fmod(t_local, interval_);
        t_mid = t - t_local_mod + interval_ * (1 + 0.5 * duty_);
        return true;
    }

    /**
     * @brief Get the Inner Event Times object
     * @note
     * Case 1:
     * eventTimes       stance      swing         stance     swing
     * ------------------------[--------------]----------[-----------]-------
     * timeBound ----------[====================================]------------
     * innerEvent -------------[==============]------------------------------
     * Output -----------------[--------------]------------------------------
     *
     * Case 2:
     * eventTimes       stance      swing         stance     swing
     * ------------------------[--------------]----------[-----------]-------
     * timeBound ----------[===========================================]-----
     * innerEvent -------------[==============]----------[===========]-------
     * Output -----------------[--------------]----------[-----------]-------
     * @param t_lb Lower bound of time
     * @param t_ub Upper bound of time
     * @param event_times Event time pairs (lift, touch)
     * @return true
     * @return false
     */
    bool getEventTimes(double t_lb, double t_ub, std::vector<std::pair<double, double>> &event_times)
    {
        event_times.clear();
        if (!is_running_)
            return false;
        double t_local_lb = t_lb - start_time_ - phase_shift_ * interval_;
        double t_local_ub = t_ub - start_time_ - phase_shift_ * interval_;

        double t_local_lb_mod = fmod(t_local_lb, interval_);
        double ts_local = t_local_lb - t_local_lb_mod;
        if (ts_local + duty_ * interval_ < t_local_lb) // if the first lift time is before t_lb
            ts_local += interval_;
        while (ts_local + interval_ < t_local_ub) // FIXME: is it appropriate?
        {
            double t_lift_local = ts_local + duty_ * interval_;
            double t_touch_local = ts_local + interval_;
            // Convert to global time
            if (t_lift_local > 0) // Make sure the first lift time is after t_lb
                event_times.emplace_back(std::make_pair(t_lift_local + start_time_ + phase_shift_ * interval_,
                                                        t_touch_local + start_time_ + phase_shift_ * interval_));
            ts_local += interval_;
        }
        return true;
    }
};

struct LegTraj
{
    double t_lift;
    double t_touch;
    double t_mid;
    // In World frame
    Eigen::Vector3d foothold_lift;
    Eigen::Vector3d foothold_touch;
    // World frame OR cfg space
    std::shared_ptr<MincoTrajectory> swing_traj;

    /**
     * @brief Construct a new Leg Traj object
     *
     * @param t_lift
     * @param t_touch
     * @param foothold_lift   In World frame
     * @param foothold_touch  In World frame
     * @param swing_traj
     */
    LegTraj(double t_lift, double t_touch,
            Eigen::Vector3d foothold_lift, Eigen::Vector3d foothold_touch,
            std::shared_ptr<MincoTrajectory> swing_traj)
        : t_lift(t_lift), t_touch(t_touch), t_mid((t_lift + t_touch) / 2),
          foothold_lift(foothold_lift), foothold_touch(foothold_touch),
          swing_traj(swing_traj){};

    [[deprecated]] void update(double t_lift, double t_touch,
                               Eigen::Vector3d foothold_lift, Eigen::Vector3d foothold_touch,
                               Eigen::Vector3d foothold_lift_cfg, Eigen::Vector3d foothold_touch_cfg,
                               Eigen::Vector3d liftvel_cfg, Eigen::Vector3d touchvel_cfg)
    {
        this->t_lift = t_lift;
        this->t_touch = t_touch;
        this->t_mid = (t_lift + t_touch) / 2;
        this->foothold_lift = foothold_lift;
        this->foothold_touch = foothold_touch;
        swing_traj->setConditions(foothold_lift_cfg, foothold_touch_cfg, liftvel_cfg, touchvel_cfg);
    }

    bool isApproxTmid(double t)
    {
        return std::abs(t - t_mid) < 1e-3;
    };

    Eigen::VectorXd evaluate(double t)
    {
        double norm_t = (t - t_lift) / (t_touch - t_lift);
        return swing_traj->evaluate(norm_t, 0, true);
    }

    bool isInDuration(double t)
    {
        return t >= t_lift && t <= t_touch;
    }
};

class CmdVelExtrapolator
{
protected:
    geometry_msgs::Twist cmd_vel_; // cmd vel relative to the BASE frame
    pinocchio::SE3 pose_;

public:
    CmdVelExtrapolator() = default;
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
    virtual pinocchio::SE3 extrapolate(double dt)
    {
        pinocchio::SE3 pose_new = pose_;
        Eigen::Vector3d linear_world;
        linear_world << cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.linear.z;
        linear_world = pose_.rotation() * linear_world;
        Eigen::Vector3d angular_world;
        angular_world << cmd_vel_.angular.x, cmd_vel_.angular.y, cmd_vel_.angular.z;
        angular_world = pose_.rotation() * angular_world;
        pinocchio::Motion angular_world_motion;
        angular_world_motion.linear() = Eigen::Vector3d::Zero();
        angular_world_motion.angular() = angular_world;

        pose_new.translation() += linear_world * dt;
        pose_new.rotation() = pose_.rotation() * pinocchio::exp6(angular_world_motion * dt).rotation();
        return pose_new;
    }
};

class GridMapCmdVelExtrapolator : public CmdVelExtrapolator
{
protected:
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    PosList exp_pose_samples_; // Expected pose sample points in base frame
    double nominal_height_ = 0.25;

public:
    void init(std::shared_ptr<GridMapInterface> gridmap_interface,
              PosList exp_pose_samples)
    {
        gridmap_interface_ = gridmap_interface;
        exp_pose_samples_ = exp_pose_samples;
    }

    pinocchio::SE3 extrapolate(double dt) override
    {
        PosList map_sample_projection;
        pinocchio::SE3 pose_new = CmdVelExtrapolator::extrapolate(dt);
        // FIXME: performance can be improved
        for (auto &pos_base : exp_pose_samples_)
        {
            auto pos_world = point_SE3Act(pose_new.inverse(), pos_base);
            pos_world(2) = gridmap_interface_->value(grid_map::Position(pos_world(0), pos_world(1)));
            map_sample_projection.emplace_back(pos_world);
        }

        // plane fitting
        Eigen::Vector3d plane;
        plane_fitting(map_sample_projection, plane);
        pose_new.translation()[2] = plane(0) * pose_new.translation()[0] + plane(1) * pose_new.translation()[1] + plane(2) + nominal_height_;
        pinocchio::Motion rot_calib_motion;
        Eigen::Vector3d body_normal = pose_.rotation().col(2);
        Eigen::Vector3d exp_normal = Eigen::Vector3d(-plane(0), -plane(1), 1);
        exp_normal.normalize();
        rot_calib_motion.angular() = body_normal.cross(exp_normal);
        pose_new.rotation() = pinocchio::exp6(rot_calib_motion).rotation() * pose_new.rotation();
        return pose_new;
    }
};

/**
 * @brief simple raibert planner without trajectory stack
 *
 */
class SimpleRaibertPlanner
{
private:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    GridMapCmdVelExtrapolator cmd_vel_extrapolator_;
    std::vector<LegSwitchScheduler> switch_scheduler_;

    // Datas
    double update_time_ = 0;
    PosList last_footholds_; // World frame
    PosList next_footholds_; // World frame
    PosList footpos_cache_;  // World frame
    pinocchio::SE3 pose_;
    geometry_msgs::Twist cmd_vel_;

    PosList nominal_foothold_base_;
    double interval_ = 1;
    double duty_ = 0.5;
    double vLift_ = 0.3;

public:
    SimpleRaibertPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                         std::shared_ptr<GridMapInterface> gridmap_interface,
                         std::shared_ptr<ElSpiderAirInterface> robot_interface);

    void start(pinocchio::SE3 pose, PosList foot_pos_list = PosList());

    void update(pinocchio::SE3 pose, geometry_msgs::Twist cmd_vel,
                PosList last_footholds = PosList());

    bool query(double t, pinocchio::SE3 &pose, PosList &foot_pos_list,
               std::array<bool, 6> &support_state);
};

class RaibertHeuristicPlanner
{
private:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    std::shared_ptr<SwingTrajPlanner> swing_traj_planner_;
    GridMapCmdVelExtrapolator cmd_vel_extrapolator_;
    std::vector<std::vector<LegTraj>> leg_traj_;
    std::vector<LegSwitchScheduler> switch_scheduler_;

    PosList nominal_foothold_base_;
    double update_time_ = 0;
    bool use_cfg_space_;

    double interval_ = 1;
    double duty_ = 0.5;
    double extrapolate_window_ = 3;

public:
    RaibertHeuristicPlanner(SwingTrajPlannerConfig swing_traj_planner_config,
                            std::shared_ptr<GridMapInterface> gridmap_interface,
                            std::shared_ptr<ElSpiderAirInterface> robot_interface);

    void start(pinocchio::SE3 pose, PosList foot_pos_list = PosList());

    void update(pinocchio::SE3 pose, geometry_msgs::Twist cmd_vel,
                PosList foot_pos_list = PosList());

    bool query(double t, pinocchio::SE3 &pose,
               PosList &foot_pos_list,
               std::array<bool, 6> &support_state);

    bool queryCfg(double t, pinocchio::SE3 &pose,
                  PosList &foot_pos_list,
                  std::array<bool, 6> &support_state);

    bool toCfgSpace(pinocchio::SE3 pose, Eigen::Vector3d pos, Eigen::Vector3d vel,
                    Eigen::Vector3d &pos_cfg, Eigen::Vector3d &vel_cfg,
                    int leg_index);
};
