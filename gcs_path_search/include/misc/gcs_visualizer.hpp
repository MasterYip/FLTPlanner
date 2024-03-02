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
#include "gcs_traj_opt/geo_utils/trajectory.hpp"
#include "gcs_traj_opt/geo_utils/quickhull.hpp"
#include "gcs_traj_opt/geo_utils/geo_utils.hpp"
#include "gcs_traj_opt/geo_utils/polyhedra.hpp"

class GCSVisualizer : public ros_visualizer::ROSVisualizer
{
public:
    GCSVisualizer(ros::NodeHandle &nh) : ROSVisualizer(nh) {};
    GCSVisualizer(ros::NodeHandle &nh, std::string frame_id, std::string topic_name) : ROSVisualizer(nh, frame_id, topic_name) {};
    ~GCSVisualizer() {};

    void visPolytope(const Eigen::Matrix3Xd &vPoly);
    void visPolytope(const std::vector<Eigen::Matrix3Xd> &vPolys);
    void visPolytope(const Eigen::MatrixX4d &hPoly);
    void visPolytope(const std::vector<Eigen::MatrixX4d> &hPolys);
    void visPolytope(std::vector<Polyhedra> &polys);
}