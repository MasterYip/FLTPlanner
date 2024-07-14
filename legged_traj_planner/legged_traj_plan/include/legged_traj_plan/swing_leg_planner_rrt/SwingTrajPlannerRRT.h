/**
 * @file SwingTrajPlannerRRT.h
 * @author Master Yip (2205929492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-07-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <memory>
/* external project header files */
#include <Eigen/Dense>
/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"
#include "legged_traj_search/utils/gcs_visualizer.hpp"

class SwingTrajPlannerRRT
{
private:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    ros::NodeHandle nh_;
    std::shared_ptr<GCSVisualizer> visualizer_;
    SwingTrajOptRRT swing_traj_opt_;

    SwingTrajPlannerConfig config_;

    std::vector<BenchmarkResult> benchmark_results_;

public:
    SwingTrajPlannerRRT(SwingTrajPlannerConfig config,
                     std::shared_ptr<ElSpiderAirInterface> robot_interface,
                     std::shared_ptr<GridMapInterface> gridmap_interface);
    ~SwingTrajPlannerRRT() = default;


    std::shared_ptr<MincoTrajectory> getDefaultTraj(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                                    double v_lift, double h_lift = 0.1);

    std::shared_ptr<MincoTrajectory> getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                       const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index,
                                                       double v_lift, double h_lift = 0.1);

    std::shared_ptr<MincoTrajectory> getInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                 Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                 double v_lift, double h_lift, uint index);

    bool getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                        pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                        Eigen::Vector3d p0, Eigen::Vector3d p1,
                        uint index);
    std::shared_ptr<MincoTrajectory> getCfgInitTraj(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                    Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                    double v_lift, uint index);

    bool optCfgTraj(std::shared_ptr<TrajectoryBase> &traj,
                    const pinocchio::SE3 &pose0,
                    const pinocchio::SE3 &pose1,
                    int index);

    bool optTraj(std::shared_ptr<TrajectoryBase> &traj,
                 const pinocchio::SE3 &pose0,
                 const pinocchio::SE3 &pose1,
                 int index);

    std::shared_ptr<ElSpiderAirInterface> getRobotInterface()
    {
        return robot_interface_;
    }

    const SwingTrajPlannerConfig &getConfig() const
    {
        return config_;
    }

    void visClear()
    {
        visualizer_->delAll();
    }

    void saveBenchmarkResults();
};