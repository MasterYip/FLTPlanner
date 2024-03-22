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
#include <vector>
#include <string>
#include <Eigen/Core>

/* external project header files */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* internal project header files */

namespace ros_visualizer
{

    struct VisStyle
    {
        double r, g, b, a;
        double x, y, z;
        VisStyle()
            : r(1.0), g(0.0), b(0.0), a(1.0), x(0.05), y(0.05), z(0.05){};
        VisStyle(double r_, double g_, double b_, double a_, double width_)
            : r(r_), g(g_), b(b_), a(a_), x(width_), y(width_), z(width_){};
        VisStyle(double r_, double g_, double b_, double a_, double x_, double y_, double z_)
            : r(r_), g(g_), b(b_), a(a_), x(x_), y(y_), z(z_){};
    };

    // const VisStyle STYLE_CURVE = VisStyle(1.0, 0.0, 0.0, 1.0, 0.004);
    // const VisStyle STYLE_MESH = VisStyle(1.0, 0.0, 0.0, 1.0, 0.002);
    // const VisStyle STYLE_FACET = VisStyle(0.0, 0.0, 1.0, 0.4, 1.0);
    // const VisStyle STYLE_SCATTER = VisStyle(0.0, 1.0, 0.0, 1.0, 0.01);
    // const VisStyle STYLE_SPHERE = VisStyle(0.0, 1.0, 0.0, 1.0, 0.01);
    // const VisStyle STYLE_CUBE = VisStyle(0.0, 1.0, 0.0, 1.0, 0.01);
    // const VisStyle STYLE_ARROW = VisStyle(0.0, 1.0, 0.0, 1.0, 0.01);

    const VisStyle STYLE_CURVE = VisStyle(1.0, 0.6, 0.002, 1.0, 0.01);
    const VisStyle STYLE_MESH = VisStyle(0.357, 0.458, 0.710, 0.3, 0.005);
    const VisStyle STYLE_FACET = VisStyle(0.15, 0.6, 0.8, 0.1, 1.0);
    const VisStyle STYLE_SCATTER = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02);
    const VisStyle STYLE_SPHERE = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02);
    const VisStyle STYLE_CUBE = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02);
    const VisStyle STYLE_ARROW = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02);

    struct VisType
    {
        std::string name_space;
        int marker_type;
        VisStyle style;
    };

    // Geometry Objects
    const VisType TYPE_CURVE = {"curve", visualization_msgs::Marker::LINE_STRIP, STYLE_CURVE};
    const VisType TYPE_FACET = {"facet", visualization_msgs::Marker::TRIANGLE_LIST, STYLE_FACET};
    const VisType TYPE_MESH = {"mesh", visualization_msgs::Marker::LINE_LIST, STYLE_MESH};

    // Scatters
    const VisType TYPE_SCATTER = {"scatter", visualization_msgs::Marker::SPHERE_LIST, STYLE_SCATTER}; // SPHERE_LIST, POINTS, CUBE_LIST
    const VisType TYPE_SPHERE = {"sphere", visualization_msgs::Marker::SPHERE_LIST, STYLE_SPHERE};
    const VisType TYPE_CUBE = {"cube", visualization_msgs::Marker::CUBE_LIST, STYLE_CUBE};

    // Arrows
    const VisType TYPE_ARROW = {"arrow", visualization_msgs::Marker::ARROW, STYLE_ARROW};

    class ROSVisualizer
    {
    private:
        std::string frame_id_ = "odom";
        std::string topic_name_ = "visualizer_markers";

        ros::NodeHandle nh_;
        ros::Publisher marker_pub_;
        visualization_msgs::MarkerArray marker_array_;

        long long marker_id_ptr_ = 0;

    public:
        ROSVisualizer(ros::NodeHandle &nh);
        ROSVisualizer(ros::NodeHandle &nh, std::string frame_id, std::string topic_name);
        ~ROSVisualizer();

        void delType(const VisType &type);
        void delAll(void);

        void visArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const VisStyle &style = TYPE_ARROW.style);
        void delArrow(void);
        void visCurve(const std::vector<Eigen::Vector3d> &curve, const VisStyle &style = TYPE_CURVE.style);
        void delCurve(void);
        void visSphere(const Eigen::Vector3d &sphere, double radius=TYPE_SPHERE.style.x, const VisStyle &style = TYPE_SPHERE.style);
        void visSphere(const std::vector<Eigen::Vector3d> &shperes, const VisStyle &style = TYPE_SPHERE.style);
        void visSphere(const std::vector<Eigen::Vector3d> &shperes, double radius, const VisStyle &style = TYPE_SPHERE.style);
        void delSphere(void);
        void visCube(const std::vector<Eigen::Vector3d> &cubes, const VisStyle &style = TYPE_CUBE.style);
        void delCube(void);
        /**
         * @brief Visualize facets
         *
         * @param facet 3 points to form a facet
         * @param style
         */
        void visFacet(const std::vector<Eigen::Vector3d> &facet, const VisStyle &style = TYPE_FACET.style);
        void visFacet(const Eigen::MatrixX3d &facet, const VisStyle &style = TYPE_FACET.style);
        void delFacet(void);

        /**
         * @brief Visualize mesh
         *
         * @param mesh 2 points to form a segment
         * @param style
         */
        void visMesh(const std::vector<Eigen::Vector3d> &mesh, const VisStyle &style = TYPE_MESH.style);
        void visMesh(const Eigen::MatrixX3d &mesh, const VisStyle &style = TYPE_MESH.style);
        void delMesh(void);
    };

} // namespace ros_visualizer