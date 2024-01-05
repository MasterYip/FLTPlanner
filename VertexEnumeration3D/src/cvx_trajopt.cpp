
/* related header files */
#include "cvx_trajopt/cvx_trajopt.h"
/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */

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

void CVX_TrajOpt::test_map()
{
    for (int i = 0; i < map_.getLayers().size(); i++)
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

        Eigen::Vector3d pos;
        Eigen::Vector2d posxy;

        pos[2] = map_.at("elevation", *iterator);
        map_.getPosition(*iterator, posxy);

        pos[0] = posxy.x();
        pos[1] = posxy.y();
        bool in_corridor = false;
        for (uint i = 0; i < CorridorBuf.size(); i++)
        {
            if (geo_utils::inVpoly(CorridorBuf.at(i), pos))
            {
                in_corridor = true;
                break;
            }
        }

        if (in_corridor)
        {
            visualizer_.visualizeSphere(pos, 0.01);
        }
    }

    return;
}
