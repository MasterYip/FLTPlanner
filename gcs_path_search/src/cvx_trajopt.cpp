
/* related header files */
#include "cvx_trajopt/cvx_trajopt.h"
/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

/* internal project header files */
#include "gcs_traj_opt/poly_traj/gcs_astar_search.hpp"
#include "gcs_traj_opt/geo_utils/geo_utils_2d.hpp"
#include "gcs_traj_opt/poly_traj/intersect_border.hpp"
#include "gcs_traj_opt/geo_utils/guide_surf.hpp"

using namespace geo_utils_2d;

CVX_TrajOpt::CVX_TrajOpt(CVX_TrajOpt_Config &conf,
                         ros::NodeHandle &nh_) : nh_(nh_), gcs_visualizer_(nh_), conf_(conf)
{
    ROS_INFO("CVX_TrajOpt::CVX_TrajOpt()");
    map_sub_ = nh_.subscribe(conf_.mapTopic, 1, &CVX_TrajOpt::map_callback, this);
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("cvx_trajopt_mappub", 1, true);
    // 并将回调函数和服务端绑定
    f = boost::bind(&CVX_TrajOpt::dyn_reconf_callback, this, _1, _2);
    server.setCallback(f);

    pos_shift = Eigen::MatrixX3d::Zero(1, 3);

    // Default CVX Hull
    Eigen::MatrixX3d FootHull(10, 3);
    FootHull << 0.2412, -0.154, -0.1303,
        -0.07939, -0.1551, -0.1464,
        -0.0809, -0.1567, -0.3889,
        0.2556, -0.1674, -0.3545,
        -0.3199, -0.3958, 0.006312,
        -0.2209, -0.2967, -0.3344,
        0.3721, -0.2772, 0.02371,
        0.3527, -0.2589, -0.2644,
        0.05979, -0.4186, -0.2857,
        0.06059, -0.472, 0.1195;
    vPoly = FootHull.transpose();

    if (!map_received_)
    {
        ROS_WARN("Waiting for map...");
        while (!map_received_ && ros::ok())
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("Map received!");
    }
}

CVX_TrajOpt::~CVX_TrajOpt()
{
}

void CVX_TrajOpt::dyn_reconf_callback(gcs_path_search::CvxTrajOptConfig &config, uint32_t level)
{
    start[0] = config.start_x;
    start[1] = config.start_y;
    pos_shift << config.pos_shift_x, config.pos_shift_y, config.pos_shift_z;
}

void CVX_TrajOpt::map_callback(const grid_map_msgs::GridMap::ConstPtr &msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    map_received_ = true;
    return;
}

Eigen::Vector2d CVX_TrajOpt::getPos(const GridPt &idx)
{
    Eigen::Vector2d posxy;
    map_.getPosition(idx, posxy);
    return posxy;
}

void CVX_TrajOpt::benchmarkInit()
{
    // Benchmark Init
    tot_time = 0;
    algo_time = 0;
    timer_.timerReset();
    ROS_INFO("==========Benchmark==========");
}

void CVX_TrajOpt::benchmarkCheck(std::string prefix, bool algo)
{
    period_time = timer_.timerCheck();
    tot_time += period_time;
    if (algo)
    {
        algo_time += period_time;
        ROS_INFO("\033[1;32m%s time:\t%f ms\033[0m", prefix.c_str(), period_time / 1e6);
    }
    else
        ROS_INFO("%s time:\t%f ms", prefix.c_str(), period_time / 1e6);
    timer_.timerReset();
}

void CVX_TrajOpt::benchmarkEnd()
{
    ROS_INFO("\033[1;31mTotal time: %f ms\033[0m", tot_time / 1e6);
    ROS_INFO("\033[1;31mAlgo time: %f ms\033[0m", algo_time / 1e6);
}

////////////////////
// GCS Path Search

// void CVX_TrajOpt::gcs_path_search(std::vector<Polyhedra> polys, Point3D start, Point3D goal)
// {

// }

////////////////////
// rope straining method

/**
 * @brief Get the min length path using `rope straining method`
 * BUG: not properly implemented
 * @param[in] Border
 * @param[in] start
 * @param[in] goal
 * @param[out] path
 * @return true
 * @return false
 */
bool CVX_TrajOpt::minlengthPath(const std::vector<GridPt> &Border,
                                const Eigen::Vector2d &start,
                                const Eigen::Vector2d &goal,
                                std::vector<GridPt> &path)
{
    GridPt start_idx, goal_idx; // FIXME: is this appropriate?
    map_.getIndex(start, start_idx);
    map_.getIndex(goal, goal_idx);
    path.clear();
    path.push_back(start_idx);
    path.push_back(goal_idx);

    // TODO: use Border!
    std::vector<GridPt> concave_points;
    findConcavePoint(Border, concave_points);
    // std::vector<GridPt> ptsSideA, ptsSideB;
    // findConcavePoint(Border, start, goal, ptsSideA, ptsSideB);
    uint i = 0, ip1; // Concave point index
    int intersect_id = -1;
    uint last_update_cnt = 0;
    uint max_seg = 20;
    while (true)
    {
        ip1 = (i + 1) % concave_points.size();
        intersect_id = pathIntersect(path, concave_points.at(i), concave_points.at(ip1));
        if (intersect_id >= 0)
        {
            printf("i: %d, ip1: %d, intersect_id: %d\n", i, ip1, intersect_id);
            printf("p1: (%d %d), p2: (%d %d)\n", concave_points.at(i)[0], concave_points.at(i)[1], concave_points.at(ip1)[0], concave_points.at(ip1)[1]);
            printf("path_i: (%d %d), path_ip1: (%d %d)\n", path.at(intersect_id)[0], path.at(intersect_id)[1], path.at(intersect_id + 1)[0], path.at(intersect_id + 1)[1]);
            last_update_cnt = 0;
            path.insert(path.begin() + intersect_id + 1, concave_points.at(ip1));
        }

        i = ip1;
        last_update_cnt++;
        if (last_update_cnt > concave_points.size())
        {
            break;
        }
        if (path.size() > max_seg)
        {
            break;
        }
    }
    return true;
}

////////////////////
// Tests

void CVX_TrajOpt::test_map()
{
    for (uint i = 0; i < map_.getLayers().size(); i++)
    {
        printf("%s\n", map_.getLayers()[i].c_str());
    }

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
        printf("%f\n", map_.at("elevation", *iterator));
    }
    return;
}

void CVX_TrajOpt::segmentIntersectTest()
{
    GridPt p1, p2, q1, q2;
    p1 << 0, 0;
    p2 << 1, 1;
    q1 << 0, 1;
    q2 << 1, 0;
    segmentIntersect(p1, p2, q1, q2, true);
    return;
}

/**
 * @brief Temporarily is used to draw the feasible connected domain & robot
 *
 */
void CVX_TrajOpt::schematic_drawer()
{
    gcs_visualizer_.delAll();
    std::vector<Eigen::Matrix3Xd> RegionBuf;
    std::vector<Eigen::Matrix3Xd> CorridorBuf;

    Eigen::MatrixX3d waypoints(3, 3);
    // For normal ElSpider Air
    waypoints << -0.2, -0.08, 0.16,
        0.0, -0.08, 0.3,
        0.2, -0.08, 0.16;

    // For Guide Surf Demo
    waypoints << -1.0, -0.5, 0,
        0.0, 0.5, 1.6,
        0.5, -0.5, 0.0;

    for (int i = 0; i < waypoints.rows(); i++)
    {
        RegionBuf.push_back((vPoly.array().colwise() + waypoints.transpose().col(i).array()).eval());
        if (i > 0)
        {
            CorridorBuf.push_back(geo_utils::mergeVpoly(RegionBuf.at(i - 1), RegionBuf.at(i)));
        }
    }

    // visualizer.visualizePolytope(RegionBuf);
    std::vector<Point3D> key_points;
    for (auto region : RegionBuf)
    {
        Polyhedra poly(region);
        key_points.push_back(poly.getInterior());
    }

    gcs_visualizer_.visPolytope(CorridorBuf);
    HarmonicGuideSurf guide_surf(key_points, 1);
    map_.add("guide_surf");
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Position pos;
        map_.getPosition(*iterator, pos);
        map_.at("guide_surf", *iterator) = guide_surf.getHeight(pos);
    }
    grid_map_msgs::GridMap gm_message;
    grid_map::GridMapRosConverter::toMessage(map_, gm_message);
    map_pub_.publish(gm_message);
    return;
}

void CVX_TrajOpt::drawCorriderIntersectBorderTest()
{
    // clean
    gcs_visualizer_.delAll();

    Eigen::MatrixX3d waypoints(3, 3);
    waypoints << -0.5, 0.0, 0.2,
        0.0, 0.4, 0.4,
        // -0.2, 0.0, 0.5,
        // 0.2, 0.0, 0.5,
        // 0.3, 0.0, 0.3,
        0.5, 0.0, 0.2;
    // Eigen::Vector2d start(-0.4, -0.3), goal(0.4, -0.3);
    Eigen::Vector2d goal(0.4, -0.3);

    GridPt start_idx, goal_idx;
    map_.getIndex(start, start_idx);
    map_.getIndex(goal, goal_idx);
    // Start
    Point3D pos;
    pos.head(2) = getPos(start_idx);
    pos[2] = map_.at("elevation", start_idx);
    gcs_visualizer_.visSphere(pos, 0.02);
    // Goal
    pos.head(2) = getPos(goal_idx);
    pos[2] = map_.at("elevation", goal_idx);
    gcs_visualizer_.visSphere(pos, 0.02);

    Eigen::Vector3d start3d, goal3d;
    start3d << start[0], start[1], map_.at("elevation", start_idx);
    goal3d << goal[0], goal[1], map_.at("elevation", goal_idx);
    gcs_visualizer_.visSphere(start3d, 0.01);
    gcs_visualizer_.visSphere(goal3d, 0.01);

    std::vector<Polyhedra> polys;
    for (int i = 0; i < waypoints.rows(); i++)
    {
        Eigen::Matrix3Xd tmpvPoly = (vPoly.array().colwise() + (waypoints.transpose().col(i).array() + pos_shift.transpose().col(0).array())).eval();
        polys.emplace_back(Polyhedra(tmpvPoly));
    }
    // gcs_visualizer_.visPolytope(polys);

    PolyCorridor poly_corridor(polys, start3d, goal3d);
    BorderCheck border_check(poly_corridor, map_, "elevation");
    IntersectBorder intersect_border(poly_corridor, border_check);
    PolyTrajSearch poly_traj_search(intersect_border);
    std::vector<Point3D> path;
    poly_traj_search.search(start3d, goal3d, path);

    // Draw VisGraph
    VisibilityGraph vis_graph = poly_traj_search.getVisGraph();
    uint size = vis_graph.size();
    for (uint i = 0; i < size; i++)
    {
        for (uint j = i + 1; j < size; j++)
        {
            if (vis_graph.isVisibile(i, j))
            {
                Point3D pos1, pos2;
                pos1.head(2) = getPos(vis_graph.getPt(i));
                pos2.head(2) = getPos(vis_graph.getPt(j));
                pos1[2] = border_check.queryHeight(vis_graph.getPt(i));
                pos2[2] = border_check.queryHeight(vis_graph.getPt(j));
                std::vector<Point3D> line;
                line.push_back(pos1);
                line.push_back(pos2);
                gcs_visualizer_.visMesh(line, ros_visualizer::VisStyle(0.3, 0.3, 0.3, 0.3, 0.01));
            }
        }
    }

    // Draw Corridor
    std::vector<Polyhedra> corridor = poly_corridor.getCorridor();
    gcs_visualizer_.visPolytope(corridor);

    // Draw Concave Points
    std::vector<Point3D> concave_pts;
    for (auto pt : poly_traj_search.getConcavePts())
    {
        Point3D pos;
        pos.head(2) = getPos(pt);
        pos[2] = border_check.queryHeight(pt);
        concave_pts.push_back(pos);
    }
    gcs_visualizer_.visSphere(concave_pts, 0.02);

    // Draw border
    GridPolyLine border = poly_traj_search.getBorder();
    std::vector<Eigen::Vector3d> border_pos;
    for (uint i = 0; i < border.size(); i++)
    {
        Eigen::Vector3d pos;
        Eigen::Vector2d posxy;
        pos[2] = border_check.queryHeight(border.at(i));
        map_.getPosition(border.at(i), posxy);
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        border_pos.push_back(pos);
    }
    gcs_visualizer_.visCurve(border_pos);

    // Draw grid_traj
    GridPolyLine grid_traj = poly_traj_search.getGridTraj();
    std::vector<Eigen::Vector3d> path_pos;
    for (uint i = 0; i < grid_traj.size(); i++)
    {
        Eigen::Vector3d pos;
        Eigen::Vector2d posxy;
        pos[2] = border_check.queryHeight(grid_traj.at(i));
        map_.getPosition(grid_traj.at(i), posxy);
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        path_pos.push_back(pos);
    }
    gcs_visualizer_.visCurve(path_pos);

}
