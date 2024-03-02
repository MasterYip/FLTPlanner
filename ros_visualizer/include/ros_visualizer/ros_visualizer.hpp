/**
 * @file ros_visualizer.hpp
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
#include <iostream>
// #include <memory>
// #include <chrono>
// #include <cmath>
#include <vector>
#include <string>
#include <Eigen/Core>

/* external project header files */

#include <ros/ros.h>
// #include <std_msgs/Float64.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* internal project header files */

namespace ros_visualizer
{

struct VisType
{
    std::string name_space;
    int marker_type;
};

// Geometry Objects
const VisType CURVE = {"curve", visualization_msgs::Marker::LINE_STRIP};
const VisType MESH = {"mesh", visualization_msgs::Marker::TRIANGLE_LIST};
const VisType MESH_EDGE = {"mesh", visualization_msgs::Marker::LINE_LIST};

// Scatters
const VisType SCATTER = {"scatter", visualization_msgs::Marker::SPHERE_LIST}; //SPHERE_LIST, POINTS, CUBE_LIST
const VisType SPHERE = {"sphere", visualization_msgs::Marker::SPHERE_LIST};
const VisType CUBE = {"cube", visualization_msgs::Marker::CUBE_LIST};

// Arrows
const VisType ARROW = {"arrow", visualization_msgs::Marker::ARROW};

struct VisStyle
{
    double r, g, b, a;
    double x, y, z;
    VisStyle()
        : r(0.0), g(0.0), b(0.0), a(1.0), x(0.01), y(0.01), z(0.01){};
    VisStyle(double r_, double g_, double b_, double a_, double width_)
        : r(r_), g(g_), b(b_), a(a_), x(width_), y(width_), z(width_){};
    VisStyle(double r_, double g_, double b_, double a_, double x_, double y_, double z_)
        : r(r_), g(g_), b(b_), a(a_), x(x_), y(y_), z(z_){};
};

class ROSVisualizer
{
private:
    std::string frame_id_ = "odom";
    std::string topic_name_ = "~visualizer_markers";

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    visualization_msgs::MarkerArray marker_array_;

    int marker_id_ptr_ = 0;

public:
    ROSVisualizer(ros::NodeHandle &nh);
    ROSVisualizer(ros::NodeHandle &nh, std::string frame_id, std::string topic_name);
    ~ROSVisualizer();

    void delType(const VisType &type);

    void visCurve(const std::vector<Eigen::Vector3d> &curve, const VisStyle &style = VisStyle());
    void delCurve(void);
    void visSphere(const std::vector<Eigen::Vector3d> &shperes, const VisStyle &style = VisStyle());
    void delSphere(void);
};

} // namespace ros_visualizer