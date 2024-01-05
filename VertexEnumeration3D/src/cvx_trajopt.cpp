
/* related header files */
#include "cvx_trajopt/cvx_trajopt.h"
/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */

bool inCorridor(const std::vector<Eigen::Matrix3Xd> &Corridor, const Eigen::Vector3d &pos)
{
    for (uint i = 0; i < Corridor.size(); i++)
    {
        if (geo_utils::inVpoly(Corridor.at(i), pos))
        {
            return true;
        }
    }
    return false;
}

bool inCorridor(const std::vector<Eigen::Matrix3Xd> &Corridor,
                const grid_map::GridMap &map,
                const grid_map::Index &idx,
                const std::string maplayer = "elevation")
{
    Eigen::Vector3d pos;
    Eigen::Vector2d posxy;
    pos[2] = map.at(maplayer, idx);
    map.getPosition(idx, posxy);
    pos[0] = posxy.x();
    pos[1] = posxy.y();
    return inCorridor(Corridor, pos);
}

CVX_TrajOpt::CVX_TrajOpt(CVX_TrajOpt_Config &conf, ros::NodeHandle &nh_) : nh_(nh_), visualizer_(nh_), conf_(conf)
{
    ROS_INFO("CVX_TrajOpt::CVX_TrajOpt()");
    map_sub_ = nh_.subscribe(conf_.mapTopic, 1, &CVX_TrajOpt::map_callback, this);
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
        while (!map_received_)
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

void CVX_TrajOpt::map_callback(const grid_map_msgs::GridMap::ConstPtr &msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    map_received_ = true;
    return;
}

void CVX_TrajOpt::drawSphereIdx(const grid_map::Index &idx, const double radius = 0.01)
{
    Eigen::Vector3d pos;
    Eigen::Vector2d posxy;
    pos[2] = map_.at("elevation", idx);
    map_.getPosition(idx, posxy);
    pos[0] = posxy.x();
    pos[1] = posxy.y();
    visualizer_.visualizeSphere(pos, radius);
    return;
}

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

void CVX_TrajOpt::draw_vpoly_2DinHullPointset()
{
    std::vector<Eigen::Matrix3Xd> RegionBuf;
    std::vector<Eigen::Matrix3Xd> CorridorBuf;

    Eigen::MatrixX3d waypoints(3, 3);
    waypoints << -0.5, 0.0, 0.2,
        0.0, 0.0, 0.5,
        0.5, 0.0, 0.2;
    for (int i = 0; i < waypoints.rows(); i++)
    {
        RegionBuf.push_back((vPoly.array().colwise() + waypoints.transpose().col(i).array()).eval());
        if (i > 0)
        {
            CorridorBuf.push_back(geo_utils::mergeVpoly(RegionBuf.at(i - 1), RegionBuf.at(i)));
        }
    }

    // visualizer.visualizePolytope(RegionBuf);
    visualizer_.visualizePolytope(CorridorBuf);

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
        if (inCorridor(CorridorBuf, map_, *iterator))
        {
            Eigen::Vector3d pos;
            Eigen::Vector2d posxy;
            pos[2] = map_.at("elevation", *iterator);
            map_.getPosition(*iterator, posxy);
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            visualizer_.visualizeSphere(pos, 0.01);
        }
    }

    return;
}

std::vector<grid_map::Index> CVX_TrajOpt::getCorriderIntersectBorder(const std::vector<Eigen::Matrix3Xd> &Corridor,
                                                                     const Eigen::Vector2d &start,
                                                                     const Eigen::Vector2d &goal,
                                                                     const std::string connectivity = "8")
{
    grid_map::Index start_idx, goal_idx, idx, start_border_idx, tmp_idx;
    map_.getIndex(start, start_idx);
    map_.getIndex(goal, goal_idx);
    std::vector<grid_map::Index> path; // TODO: Use freeman chain code to represent path
    idx = start_idx;
    if (!inCorridor(Corridor, map_, idx))
    {
        ROS_ERROR("Start point not in corridor!");
        return path;
    }

    while (inCorridor(Corridor, map_, idx))
    {
        idx[0]++;
    }
    start_border_idx = idx;
    path.push_back(start_border_idx);
    // Start at border
    // int[8][2] c8_ccw = {{1, 0}, {1, 1}, {0, 1}, {-1, 1},
    //                    {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    // Connectivity 8 Clockwise
    // 6 7 8
    // 5 * 1
    // 4 3 2
    // int c8_cw[9][2] = {{1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}};
    std::vector<grid_map::Index> c8_cw = {{1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}};

    do
    {
        bool in_corridor_flag = false;
        for (int i = 0; i < 9; i++)
        {
            tmp_idx = idx + c8_cw.at(i);
            // Make use of short-circuit evaluation
            if (!in_corridor_flag && inCorridor(Corridor, map_, tmp_idx))
            {
                in_corridor_flag = true;
            }
            if (in_corridor_flag && !inCorridor(Corridor, map_, tmp_idx))
            {
                path.push_back(tmp_idx);
                idx = tmp_idx;
                break;
            }
        }
    } while (idx[0] != start_border_idx[0] || idx[1] != start_border_idx[1]);
    return path;
}

void CVX_TrajOpt::drawCorriderIntersectBorder(const std::vector<Eigen::Matrix3Xd> &Corridor,
                                              const Eigen::Vector2d &start, const Eigen::Vector2d &goal)
{
    // Visualize start & goal
    grid_map::Index start_idx, goal_idx;
    map_.getIndex(start, start_idx);
    drawSphereIdx(start_idx, 0.02);
    map_.getIndex(goal, goal_idx);
    drawSphereIdx(goal_idx, 0.02);

    // Get path
    std::vector<grid_map::Index> path = getCorriderIntersectBorder(Corridor, start, goal);

    // Draw path
    std::vector<Eigen::Vector3d> path_pos;
    for (uint i = 0; i < path.size(); i++)
    {
        // drawSphereIdx(path.at(i));
        Eigen::Vector3d pos;
        Eigen::Vector2d posxy;
        pos[2] = map_.at("elevation", path.at(i));
        map_.getPosition(path.at(i), posxy);
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        path_pos.push_back(pos);
    }
    visualizer_.visualizeCurve(path_pos);
}

void CVX_TrajOpt::drawCorriderIntersectBorderTest()
{
    Eigen::MatrixX3d waypoints(3, 3);
    waypoints << -0.5, 0.0, 0.2,
        0.0, 0.0, 0.2,
        0.5, 0.0, 0.2;
    Eigen::Vector2d start(-0.4, -0.3), goal(0.4, -0.3);

    std::vector<Eigen::Matrix3Xd> RegionBuf;
    std::vector<Eigen::Matrix3Xd> CorridorBuf;

    for (int i = 0; i < waypoints.rows(); i++)
    {
        RegionBuf.push_back((vPoly.array().colwise() + waypoints.transpose().col(i).array()).eval());
        if (i > 0)
        {
            CorridorBuf.push_back(geo_utils::mergeVpoly(RegionBuf.at(i - 1), RegionBuf.at(i)));
        }
    }
    // visualizer.visualizePolytope(RegionBuf);
    visualizer_.visualizePolytope(CorridorBuf);

    drawCorriderIntersectBorder(CorridorBuf, start, goal);
}