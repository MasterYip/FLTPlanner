/**
 * @file FLTPlanner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <iostream>
#include "legged_traj_plan/swing_traj_planner/flt_planner/FLTPlanner.h"
#include "legged_traj_plan/swing_traj_planner/flt_planner/LeggedBorderCheck.h"
#include "legged_traj_plan/utils/Geometry.h"
#include "legged_traj_search/poly_traj/poly_traj_search.hpp"

Eigen::VectorXd getTrajTimeVec(const std::vector<Point3D> &path, double total_time)
{
    if (path.size() > 2)
    {
        double path_length = 0;
        Eigen::VectorXd ts(path.size() - 1);
        for (size_t i = 1; i < path.size() - 1; i++)
            path_length += (path[i] - path[i - 1]).norm();
        path_length += (path.back() - path[path.size() - 2]).norm();
        for (size_t i = 1; i < path.size(); i++)
            ts[i - 1] = (path[i] - path[i - 1]).norm() / path_length * total_time;
        return ts;
    }
    else
    {
        Eigen::VectorXd ts(1);
        ts << total_time;
        return ts;
    }
}

////////////////////
// FLTPlanner

FLTPlanner::FLTPlanner(SwingTrajPlannerConfig config,
                       std::shared_ptr<ElSpiderAirInterface> robot_interface,
                       std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                              swing_traj_opt_(robot_interface_, gridmap_interface_,
                                                                                              nullptr, config.enableBenchmark)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
    if (config_.enableOptVis)
        swing_traj_opt_.setVisualizer(visualizer_);
}

std::shared_ptr<MincoTrajectory> FLTPlanner::getDefaultTraj(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                                            double v_lift, double h_lift)
{
    // Minco
    Eigen::Vector3d start_vel = Eigen::Vector3d(0, 0, v_lift);
    Eigen::Vector3d goal_vel = Eigen::Vector3d(0, 0, -v_lift);
    std::vector<Point3D> poly_path;
    poly_path.emplace_back(p0);
    poly_path.emplace_back((p0 + p1) / 2 + Eigen::Vector3d(0, 0, h_lift));
    poly_path.emplace_back(p1);
    return std::make_shared<MincoTrajectory>(poly_path, start_vel, goal_vel, config_.trajTime);
}

bool FLTPlanner::searchPolyTraj(std::vector<Point3D> &poly_traj,
                                const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                                const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                                uint index, bool verbose)
{
    poly_traj.clear();
    // gridmap_interface_->lockMapUpdate();
    Eigen::Matrix3Xd hull = robot_interface_->getFootPolyhedra(index).getVRep();
    std::vector<Polyhedra> hulls;
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose0.inverse(), hull))));
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose1.inverse(), hull))));
    PolyCorridor corridor(hulls, p0, p1);
    PolyTrajSearch poly_traj_search(corridor, gridmap_interface_->getMap(),
                                    gridmap_interface_->getGroundLayerName(),
                                    gridmap_interface_->getCeilingLayerName(), true, false);
    if (config_.enableVis)
    {
        // Polytope
        visualizer_->setIdGroup(1);
        visualizer_->visPolytope(corridor.getCorridor());
    }

    if (!poly_traj_search.endpointValid(p0, p1))
    {
        if (verbose)
            std::cout << "Warning: poly_traj_search.endpointValid failed (leg " << index << ")" << std::endl;
        return false;
    }
    if (!poly_traj_search.reachable(p0, p1))
    {
        if (verbose)
            std::cout << "Warning: poly_traj_search.reachable failed" << std::endl;
        return false;
    }
    if (!poly_traj_search.search(p0, p1, poly_traj))
    {
        // BUG: if is reachable then it must be able to find a path, this failure should not happen
        if (verbose)
            std::cout << "Warning: poly_traj_search.search failed" << std::endl;
        return false;
    }
    // gridmap_interface_->unlockMapUpdate();

    if (config_.enableVis)
    {
        visualizer_->setIdGroup(1);
        // Polytope
        // visualizer_->visPolytope(corridor.getCorridor());
        // Start Goal
        visualizer_->visSphere(p0, 0.02);
        visualizer_->visSphere(p1, 0.02);
        // Border
        GridPolyLine border = poly_traj_search.getFullResBorder();
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy = poly_traj_search.getIndexRemap().grid2Pos(border.at(i));
            pos[2] = poly_traj_search.getBorderCheck()->queryHeight(border.at(i));
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            border_pos.emplace_back(pos);
        }
        border_pos.push_back(border_pos.front());
        visualizer_->visCurve(border_pos, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.01));
        // Poly Path
        visualizer_->visCurve(poly_traj, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.02));
    }

    return true;
}

std::shared_ptr<TrajectoryBase> FLTPlanner::getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                            Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                            uint index)
{
    double v_lift = config_.vLift;
    double h_lift = config_.hLift;
    std::vector<Point3D> poly_path;
    if (!searchPolyTraj(poly_path, pose0, pose1, p0, p1, index))
    {
        return getDefaultTraj(p0, p1, v_lift, h_lift);
    }
    // Start & End Vel in World Frame
    Eigen::Vector3d normal = gridmap_interface_->sdfDerivative(p0, 0);
    normal.normalize();
    Eigen::Vector3d start_vel = normal * v_lift;
    normal = gridmap_interface_->sdfDerivative(p1, 0);
    normal.normalize();
    Eigen::Vector3d goal_vel = -normal * v_lift;
    MincoTrajectory minco_traj(poly_path, start_vel, goal_vel, config_.trajTime);
    if (config_.enableVis)
    {
        // Minco
        visualizer_->setIdGroup(1);
        std::vector<Point3D> poly_path_opt;
        minco_traj.getTrajSamples(poly_path_opt);
        visualizer_->visCurve(poly_path_opt);
    }

    return std::make_shared<MincoTrajectory>(minco_traj);
}

bool FLTPlanner::optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                             const pinocchio::SE3 &pose0,
                             const pinocchio::SE3 &pose1,
                             int index)
{
    if (!config_.enableOptimizer)
        return true;
    std::shared_ptr<MincoTrajectory> minco_traj = std::dynamic_pointer_cast<MincoTrajectory>(traj);
    std::vector<Point3D> poly_path;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d goal_vel;
    minco_traj->getInitCondition(poly_path, start_vel, goal_vel);
    Eigen::Matrix3Xd poly_path_mat(3, poly_path.size());
    for (size_t i = 0; i < poly_path.size(); i++)
        poly_path_mat.col(i) = poly_path[i];

    swing_traj_opt_.setup(pose0, pose1, index, poly_path_mat, start_vel, goal_vel,
                          config_, false);
    bool ret = swing_traj_opt_.optimize(minco_traj->getTraj(), config_.relCostTol);

    if (config_.enableVis && ret)
    {
        // Minco
        std::vector<Point3D> path_opt;
        double ts = 0.01;
        double t = 0;
        minco_traj->getTrajSamples(path_opt, ts);
        visualizer_->setIdGroup(1);
        visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 0.5, 0.01));
    }
    return ret;
}

////////////////////
// FLTCfgPlanner

/**
 * @brief Orthogonal disk randomize
 * @note Return a randomized vector \bar{r} given a normal vector \bar{n}, the \bar{r} is orthogonal to \bar{n}
 * @param normal
 * @param radius
 * @return Eigen::Vector3d
 */
Eigen::Vector3d orthogonalDiskRandomize(const Eigen::Vector3d &normal, double radius)
{
    Eigen::Vector3d random = Eigen::Vector3d::Random();
    random.normalize();
    Eigen::Vector3d tangent = random - random.dot(normal) * normal;
    return radius * tangent;
}

FLTCfgPlanner::FLTCfgPlanner(SwingTrajPlannerConfig config,
                             std::shared_ptr<ElSpiderAirInterface> robot_interface,
                             std::shared_ptr<GridMapInterface> gridmap_interface) : SwingTrajPlannerBase(config, robot_interface, gridmap_interface),
                                                                                    swing_traj_opt_(robot_interface_, gridmap_interface_,
                                                                                                    nullptr, false)
{
    visualizer_ = std::make_shared<GCSVisualizer>(nh_, "odom", "swing_traj_planner_vis");
    if (config_.enableOptVis)
        swing_traj_opt_.setVisualizer(visualizer_);
}

void FLTCfgPlanner::visCfgMincoTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1, int index,
                                    std::vector<Point3D> cfg_poly_traj,
                                    Eigen::Vector3d start_vel, Eigen::Vector3d goal_vel, double trajTime,
                                    int groupId)
{
    MincoTrajectory minco_traj(cfg_poly_traj, start_vel, goal_vel, trajTime);
    std::vector<Point3D> cfg_path_opt;
    std::vector<Point3D> path_opt;
    double ts = 0.01;
    double t = 0;
    minco_traj.getTrajSamples(cfg_path_opt, ts);
    for (auto pt : cfg_path_opt)
    {
        Point3D base_pt = robot_interface_->FK_foot(pt, index);
        path_opt.emplace_back(point_SE3Act(poseLinearInterp(pose0, pose1, t).inverse(), base_pt));
        t += ts;
    }
    visualizer_->setIdGroup(groupId);
    visualizer_->visCurve(path_opt);
}

std::shared_ptr<MincoTrajectory> FLTCfgPlanner::getDefaultCfgTraj(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
                                                                  const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index)
{
    std::vector<Point3D> cfg_poly_traj;
    std::vector<Point3D> poly_path;
    poly_path.emplace_back(p0);
    poly_path.emplace_back((p0 + p1) / 2 + Eigen::Vector3d(0, 0, config_.hLift));
    poly_path.emplace_back(p1);
    // Convert to config space
    Eigen::VectorXd t_vec = getTrajTimeVec(poly_path, 1.0);
    double t = 0;
    for (uint i = 0; i < poly_path.size() - 1; i++)
    {
        // Convert to base frame
        Point3D base_pt = point_SE3Act(poseLinearInterp(pose0, pose1, t), poly_path[i]);
        cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));
        t += t_vec(i);
    }
    Point3D base_pt = point_SE3Act(pose1, poly_path.back());
    cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));

    // Get start and goal velocity in config space
    // NOTE: the vel is in BASE frame, not in WORLD frame
    Eigen::Vector3d normal = gridmap_interface_->sdfDerivative(p0, 0);
    normal.normalize();
    if (config_.enableLiftRandomize)
        normal += orthogonalDiskRandomize(normal, config_.vLiftNormalRandomize);
    Eigen::Vector3d start_vel = vec_SE3Act(pose0, normal * config_.vLift); // Base frame
    normal = gridmap_interface_->sdfDerivative(p1, 0);
    normal.normalize();
    if (config_.enableLiftRandomize)
        normal += orthogonalDiskRandomize(normal, config_.vLiftNormalRandomize);
    Eigen::Vector3d goal_vel = vec_SE3Act(pose1, -normal * config_.vLift); // Base frame
    Eigen::Matrix3Xd J = robot_interface_->getJacobian(cfg_poly_traj.front(), index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    start_vel = J_inv * start_vel;
    J = robot_interface_->getJacobian(cfg_poly_traj.back(), index);
    J_inv = J.transpose() * (J * J.transpose()).inverse();
    goal_vel = J_inv * goal_vel;

    if (config_.enableVis)
    {
        visCfgMincoTraj(pose0, pose1, index, cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
    }

    return std::make_shared<MincoTrajectory>(cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
}

bool FLTCfgPlanner::searchPolyTrajPITD(std::vector<Point3D> &poly_traj,
                                       const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                                       const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                                       uint index, bool verbose)
{
    poly_traj.clear();
    // gridmap_interface_->lockMapUpdate();
    std::unique_ptr<PolyTrajSearch> poly_traj_search;

    // Use LeggedBorderCheck
    LeggedBorderCheckConfig config;
    config.ground_layer = gridmap_interface_->getGroundLayerName();
    config.ceiling_layer = gridmap_interface_->getCeilingLayerName();
    config.enable_ground = true;
    config.enable_ceiling = false;
    config.collBallRad1 = config_.CollBall1Rad;
    config.collBallRad2 = config_.CollBall2Rad;
    config.collBallRad3 = config_.CollBall3Rad;
    auto border_check = std::make_shared<LeggedBorderCheck>(robot_interface_, gridmap_interface_,
                                                            pose0, pose1, p0, p1, index, config);
    PolyTrajSearchConfig cfg;
    cfg.enable_benchmark = false;
    poly_traj_search = std::make_unique<PolyTrajSearch>(border_check, gridmap_interface_->getMap(), cfg);

    bool ret_endpoint = poly_traj_search->endpointValid(p0, p1);
    bool ret_reachable = poly_traj_search->reachable(p0, p1);
    bool ret_search = poly_traj_search->search(p0, p1, poly_traj);
    // gridmap_interface_->unlockMapUpdate();

    if (verbose)
    {
        if (!ret_endpoint)
            std::cout << "Warning: poly_traj_search->endpointValid failed (leg " << index << ")" << std::endl;
        if (!ret_reachable)
            std::cout << "Warning: poly_traj_search->reachable failed" << std::endl;
        if (!ret_search)
            std::cout << "Warning: poly_traj_search->search failed" << std::endl;
        // BUG: if is reachable then it must be able to find a path, this failure should not happen
    }

    if (config_.enableVis)
    {
        // Polytope
        visualizer_->setIdGroup(1);
        // visualizer_->visPolytope(corridor.getCorridor());
        visualizer_->visSphere(p0, 0.02);
        visualizer_->visSphere(p1, 0.02);
        // Border
        GridPolyLine border = poly_traj_search->getFullResBorder();
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy = poly_traj_search->getIndexRemap().grid2Pos(border.at(i));
            pos[2] = poly_traj_search->getBorderCheck()->queryHeight(border.at(i));
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            border_pos.emplace_back(pos);
        }
        if (border_pos.size() > 0)
        {
            border_pos.push_back(border_pos.front());
            visualizer_->visCurve(border_pos, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.01));
        }
        // // Vis Graph
        // VisibilityGraph vis_graph = poly_traj_search->getVisGraph();
        // CorridorBorderCheck border_check = poly_traj_search->getBorderCheck();
        // std::vector<Point3D> mesh;
        // uint size = vis_graph.size();
        // Point3D pos1, pos2;
        // for (uint i = 0; i < 1; i++)
        // {
        //     for (uint j = 0; j < size; j++)
        //     {
        //         if (i != j && vis_graph.isVisibile(i, j))
        //         {
        //             // pos1.head(2) = getPos(vis_graph.getPt(i));
        //             // pos2.head(2) = getPos(vis_graph.getPt(j));
        //             pos1.head(2) = border_check.getIndexRemap().grid2Pos(vis_graph.getPt(i));
        //             pos2.head(2) = border_check.getIndexRemap().grid2Pos(vis_graph.getPt(j));
        //             pos1[2] = border_check.queryHeight(vis_graph.getPt(i));
        //             pos2[2] = border_check.queryHeight(vis_graph.getPt(j));
        //             mesh.push_back(pos1);
        //             mesh.push_back(pos2);
        //         }
        //     }
        // }
        // visualizer_->visMesh(mesh, ros_visualizer::VisStyle(0.3, 0.3, 0.9, 0.6, 0.005));
        // Poly Path
        visualizer_->visCurve(poly_traj, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.02));
    }

    return ret_endpoint && ret_reachable && ret_search;
}

bool FLTCfgPlanner::searchPolyTraj(std::vector<Point3D> &poly_traj,
                                   const pinocchio::SE3 pose0, const pinocchio::SE3 pose1,
                                   const Eigen::Vector3d p0, const Eigen::Vector3d p1,
                                   uint index, bool verbose)
{
    poly_traj.clear();
    // gridmap_interface_->lockMapUpdate();
    std::unique_ptr<PolyTrajSearch> poly_traj_search;

    // Use CorridorBorderCheck
    Eigen::Matrix3Xd hull = robot_interface_->getFootPolyhedra(index).getVRep();
    std::vector<Polyhedra> hulls;
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose0.inverse(), hull))));
    hulls.emplace_back(Polyhedra(Eigen::Matrix3Xd(points_SE3Act(pose1.inverse(), hull))));
    PolyCorridor corridor(hulls, p0, p1);
    poly_traj_search = std::make_unique<PolyTrajSearch>(corridor, gridmap_interface_->getMap(),
                                                        gridmap_interface_->getGroundLayerName(),
                                                        gridmap_interface_->getCeilingLayerName(), true, false);

    bool ret_endpoint = poly_traj_search->endpointValid(p0, p1);
    bool ret_reachable = poly_traj_search->reachable(p0, p1);
    bool ret_search = poly_traj_search->search(p0, p1, poly_traj);
    // gridmap_interface_->unlockMapUpdate();

    if (verbose)
    {
        if (!ret_endpoint)
            std::cout << "Warning: poly_traj_search->endpointValid failed (leg " << index << ")" << std::endl;
        if (!ret_reachable)
            std::cout << "Warning: poly_traj_search->reachable failed" << std::endl;
        if (!ret_search)
            std::cout << "Warning: poly_traj_search->search failed" << std::endl;
        // BUG: if is reachable then it must be able to find a path, this failure should not happen
    }

    if (config_.enableVis)
    {
        // Polytope
        visualizer_->setIdGroup(1);
        visualizer_->visPolytope(corridor.getCorridor());
        visualizer_->visSphere(p0, 0.02);
        visualizer_->visSphere(p1, 0.02);
        // Border
        GridPolyLine border = poly_traj_search->getFullResBorder();
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy = poly_traj_search->getIndexRemap().grid2Pos(border.at(i));
            pos[2] = poly_traj_search->getBorderCheck()->queryHeight(border.at(i));
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            border_pos.emplace_back(pos);
        }
        if (border_pos.size() > 0)
        {
            border_pos.push_back(border_pos.front());
            visualizer_->visCurve(border_pos, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.01));
        }
        // // Vis Graph
        // VisibilityGraph vis_graph = poly_traj_search->getVisGraph();
        // CorridorBorderCheck border_check = poly_traj_search->getBorderCheck();
        // std::vector<Point3D> mesh;
        // uint size = vis_graph.size();
        // Point3D pos1, pos2;
        // for (uint i = 0; i < 1; i++)
        // {
        //     for (uint j = 0; j < size; j++)
        //     {
        //         if (i != j && vis_graph.isVisibile(i, j))
        //         {
        //             // pos1.head(2) = getPos(vis_graph.getPt(i));
        //             // pos2.head(2) = getPos(vis_graph.getPt(j));
        //             pos1.head(2) = border_check.getIndexRemap().grid2Pos(vis_graph.getPt(i));
        //             pos2.head(2) = border_check.getIndexRemap().grid2Pos(vis_graph.getPt(j));
        //             pos1[2] = border_check.queryHeight(vis_graph.getPt(i));
        //             pos2[2] = border_check.queryHeight(vis_graph.getPt(j));
        //             mesh.push_back(pos1);
        //             mesh.push_back(pos2);
        //         }
        //     }
        // }
        // visualizer_->visMesh(mesh, ros_visualizer::VisStyle(0.3, 0.3, 0.9, 0.6, 0.005));
        // Poly Path
        visualizer_->visCurve(poly_traj, ros_visualizer::VisStyle(0.1, 0.1, 0.1, 0.5, 0.02));
    }

    return ret_endpoint && ret_reachable && ret_search;
}

bool FLTCfgPlanner::getCfgPolyTraj(std::vector<Point3D> &cfg_poly_traj,
                                   pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                   Eigen::Vector3d p0, Eigen::Vector3d p1,
                                   uint index)
{
    cfg_poly_traj.clear();
    std::vector<Point3D> poly_path;
    if ((!config_.useLeggedBorderCheck && !searchPolyTraj(poly_path, pose0, pose1, p0, p1, index)) ||
        (config_.useLeggedBorderCheck && !searchPolyTrajPITD(poly_path, pose0, pose1, p0, p1, index)))
        return false;
    // Convert to config space
    Eigen::VectorXd t_vec = getTrajTimeVec(poly_path, 1.0);
    double t = 0;
    for (uint i = 0; i < poly_path.size() - 1; i++)
    {
        // Convert to base frame
        Point3D base_pt = point_SE3Act(poseLinearInterp(pose0, pose1, t), poly_path[i]);
        cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));
        t += t_vec(i);
    }
    Point3D base_pt = point_SE3Act(pose1, poly_path.back());
    cfg_poly_traj.emplace_back(robot_interface_->IKFast_foot(base_pt, index));
    return true;
}

/**
 * @brief
 *
 * @param pose0
 * @param pose1
 * @param p0 In World frame
 * @param p1 In World frame
 * @param v_lift
 * @param index
 * @return std::shared_ptr<MincoTrajectory>
 */
std::shared_ptr<TrajectoryBase> FLTCfgPlanner::getInitTrajHook(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                                               Eigen::Vector3d p0, Eigen::Vector3d p1,
                                                               uint index)
{
    std::vector<Point3D> cfg_poly_traj;
    if (!config_.enablePolyPathSearch || !getCfgPolyTraj(cfg_poly_traj, pose0, pose1, p0, p1, index))
    {
        return getDefaultCfgTraj(pose0, pose1, p0, p1, index);
    }

    // Get start and goal velocity in config space
    // NOTE: the vel is in BASE frame
    Eigen::Vector3d normal = gridmap_interface_->sdfDerivative(p0, 0);
    normal.normalize();
    if (config_.enableLiftRandomize)
        normal += orthogonalDiskRandomize(normal, config_.vLiftNormalRandomize);
    Eigen::Vector3d start_vel = vec_SE3Act(pose0, normal * config_.vLift); // Base frame
    normal = gridmap_interface_->sdfDerivative(p1, 0);
    normal.normalize();
    if (config_.enableLiftRandomize)
        normal += orthogonalDiskRandomize(normal, config_.vLiftNormalRandomize);
    Eigen::Vector3d goal_vel = vec_SE3Act(pose1, -normal * config_.vLift); // Base frame
    Eigen::Matrix3Xd J = robot_interface_->getJacobian(cfg_poly_traj.front(), index);
    Eigen::Matrix3Xd J_inv = J.transpose() * (J * J.transpose()).inverse();
    start_vel = J_inv * start_vel;
    J = robot_interface_->getJacobian(cfg_poly_traj.back(), index);
    J_inv = J.transpose() * (J * J.transpose()).inverse();
    goal_vel = J_inv * goal_vel;
    // start_vel[2] += 0.05;
    // goal_vel[2] -= 0.05;
    MincoTrajectory minco_traj(cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
    if (config_.enableVis)
    {
        visCfgMincoTraj(pose0, pose1, index, cfg_poly_traj, start_vel, goal_vel, config_.trajTime);
    }
    return std::make_shared<MincoTrajectory>(minco_traj);
}

bool FLTCfgPlanner::optTrajHook(std::shared_ptr<TrajectoryBase> &traj,
                                const pinocchio::SE3 &pose0,
                                const pinocchio::SE3 &pose1,
                                int index)
{
    if (!config_.enableOptimizer)
        return true;
    std::shared_ptr<MincoTrajectory> minco_traj = std::dynamic_pointer_cast<MincoTrajectory>(traj);
    std::vector<Point3D> poly_path;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d goal_vel;
    minco_traj->getOptInitCondition(poly_path, start_vel, goal_vel, config_.lengthPerPiece);
    // minco_traj->getInitCondition(poly_path, start_vel, goal_vel);
    Eigen::Matrix3Xd poly_path_mat(3, poly_path.size());
    for (size_t i = 0; i < poly_path.size(); i++)
        poly_path_mat.col(i) = poly_path[i];

    swing_traj_opt_.setup(pose0, pose1, index, poly_path_mat, start_vel, goal_vel,
                          config_, true);
    bool ret = swing_traj_opt_.optimize(minco_traj->getTraj(), config_.relCostTol);
    if (config_.enableSpaceDeform)
        minco_traj->setSpaceDeform(Eigen::Vector3d(config_.spaceDeform1, config_.spaceDeform2, config_.spaceDeform3));
    else
        minco_traj->unsetSpaceDeform();

    // SwingTrajOpt Benchmark disabled for now
    // if (config_.enableBenchmark)
    // {
    //     benchmark_results_.emplace_back(swing_traj_opt_.getBenchmarkResult());
    // }

    if (config_.enableVis && ret)
    {
        // Minco
        std::vector<Point3D> cfg_path_opt;
        std::vector<Point3D> path_opt;
        double ts = 0.01;
        double t = 0;
        minco_traj->getTrajSamples(cfg_path_opt, ts);

        for (auto pt : cfg_path_opt)
        {
            Point3D base_pt = robot_interface_->FK_foot(pt, index);
            path_opt.emplace_back(point_SE3Act(poseLinearInterp(pose0, pose1, t).inverse(), base_pt));
            t += ts;
        }
        visualizer_->setIdGroup(1);
        visualizer_->visCurve(path_opt, ros_visualizer::VisStyle(1.0, 0.1, 0.1, 0.5, 0.01));
    }
    return ret;
}

// std::unique_ptr<PolyTrajSearch> FLTCfgPlanner::getPolyTrajSearch(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
//                                                                  Eigen::Vector3d p0,
//                                                                  uint index)
// {
//     // Use LeggedBorderCheck
//     LeggedBorderCheckConfig config;
//     config.ground_layer = gridmap_interface_->getGroundLayerName();
//     config.ceiling_layer = gridmap_interface_->getCeilingLayerName();
//     config.enable_ground = true;
//     config.enable_ceiling = false;
//     config.collBallRad1 = config_.CollBall1Rad;
//     config.collBallRad2 = config_.CollBall2Rad;
//     config.collBallRad3 = config_.CollBall3Rad;
//     auto border_check = std::make_shared<LeggedBorderCheck>(robot_interface_, gridmap_interface_,
//                                                             pose0, pose1, p0, p0, index, config);
//     PolyTrajSearchConfig cfg;
//     cfg.enable_benchmark = false;
//     return std::make_unique<PolyTrajSearch>(border_check, gridmap_interface_->getMap(), cfg);
// }

bool FLTCfgPlanner::reachableFilter(pinocchio::SE3 pose0, pinocchio::SE3 pose1,
                                    Eigen::Vector3d p0, uint index,
                                    std::vector<Eigen::Vector3d> &footholds,
                                    std::vector<bool> &reachable)
{
    std::unique_ptr<PolyTrajSearch> poly_traj_search;
    // Use LeggedBorderCheck
    LeggedBorderCheckConfig config;
    config.ground_layer = gridmap_interface_->getGroundLayerName();
    config.ceiling_layer = gridmap_interface_->getCeilingLayerName();
    config.enable_ground = true;
    config.enable_ceiling = false;
    config.collBallRad1 = config_.CollBall1Rad;
    config.collBallRad2 = config_.CollBall2Rad;
    config.collBallRad3 = config_.CollBall3Rad;
    auto border_check = std::make_shared<LeggedBorderCheck>(robot_interface_, gridmap_interface_,
                                                            pose0, pose1, p0, p0 + pose1.translation() - pose0.translation(), index, config);
    PolyTrajSearchConfig cfg;
    cfg.enable_benchmark = false;
    poly_traj_search = std::make_unique<PolyTrajSearch>(border_check, gridmap_interface_->getMap(), cfg);
    poly_traj_search->reachable(p0, p0); // Update intersect border

    reachable.resize(footholds.size(), false);
    for (size_t i = 0; i < footholds.size(); i++)
    {
        if (isnan(footholds[i][2]) || isnan(footholds[i][0]) || isnan(footholds[i][1]))
            continue;
        reachable[i] = poly_traj_search->reachable(p0, footholds[i], false);
    }
    return true;
}