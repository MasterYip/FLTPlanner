/**
 * @file gcs_visualizer.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "ros_visualizer/ros_visualizer.hpp"
/* internal project header files */
#include "legged_traj_search/geo_utils/trajectory.hpp"
#include "legged_traj_search/geo_utils/quickhull.hpp"
#include "legged_traj_search/geo_utils/geo_utils.hpp"
#include "legged_traj_search/geo_utils/polyhedra.hpp"

class GCSVisualizer : public ros_visualizer::ROSVisualizer
{
public:
    GCSVisualizer(ros::NodeHandle &nh, std::string frame_id = "odom", std::string topic_name = "visualizer_markers");
    ~GCSVisualizer() = default;

    void visPolytope(const Eigen::Matrix3Xd &vPoly);
    void visPolytope(const std::vector<Eigen::Matrix3Xd> &vPolys);
    void visPolytope(const Eigen::MatrixX4d &hPoly);
    void visPolytope(const std::vector<Eigen::MatrixX4d> &hPolys);
    void visPolytope(std::vector<Polyhedra> &polys);
    void visPolytope(Polyhedra &poly);
    void visPolytope(Polyhedra poly);
};
