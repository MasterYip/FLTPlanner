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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

/* internal project header files */

namespace ros_visualizer
{

    struct VisStyle
    {
        double r, g, b, a;
        double x, y, z;
        VisStyle()
            : r(1.0), g(0.0), b(0.0), a(1.0), x(0.05), y(0.05), z(0.05) {};
        VisStyle(double r_, double g_, double b_, double a_, double width_)
            : r(r_), g(g_), b(b_), a(a_), x(width_), y(width_), z(width_) {};
        VisStyle(double r_, double g_, double b_, double a_, double x_, double y_, double z_)
            : r(r_), g(g_), b(b_), a(a_), x(x_), y(y_), z(z_) {};
    };

    // Default Styles
    const VisStyle STYLE_CURVE = VisStyle(1.0, 0.6, 0.002, 1.0, 0.01);
    const VisStyle STYLE_MESH = VisStyle(0.357, 0.458, 0.710, 0.3, 0.005);
    const VisStyle STYLE_FACET = VisStyle(0.15, 0.6, 0.8, 0.1, 1.0);
    const VisStyle STYLE_SCATTER = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02);
    const VisStyle STYLE_SPHERE = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02);
    const VisStyle STYLE_CUBE = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02);
    const VisStyle STYLE_ARROW = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02);
    const VisStyle STYLE_ARROW2 = VisStyle(0.8, 0.45, 0.8, 1.0, 0.04);

    struct VisType
    {
        std::string name_space;
        int marker_type;
        VisStyle style;
    };

    struct ROSVisualizerConfig
    {
        std::string frame_id;
        std::string topic_name;
        // Geometry Objects
        VisType TYPE_CURVE = {"curve", visualization_msgs::Marker::LINE_STRIP, STYLE_CURVE};
        VisType TYPE_FACET = {"facet", visualization_msgs::Marker::TRIANGLE_LIST, STYLE_FACET};
        VisType TYPE_MESH =  {"mesh", visualization_msgs::Marker::LINE_LIST, STYLE_MESH};
        // Scatters
        VisType TYPE_SCATTER = {"scatter", visualization_msgs::Marker::SPHERE_LIST, STYLE_SCATTER}; // SPHERE_LIST, POINTS, CUBE_LIST
        VisType TYPE_SPHERE = {"sphere", visualization_msgs::Marker::SPHERE_LIST, STYLE_SPHERE};
        VisType TYPE_CUBE = {"cube", visualization_msgs::Marker::CUBE_LIST, STYLE_CUBE};
        // Arrows
        VisType TYPE_ARROW = {"arrow", visualization_msgs::Marker::ARROW, STYLE_ARROW};
        VisType TYPE_ARROW2 = {"arrow", visualization_msgs::Marker::ARROW, STYLE_ARROW2};

        void loadParams(ros::NodeHandle &nh)
        {
            std::vector<double> style;
            nh.param<std::string>("frame_id", frame_id, "odom");
            nh.param<std::string>("topic_name", topic_name, "visualizer_markers");
            nh.param("STYLE_CURVE", style, {1.0, 0.6, 0.002, 1.0, 0.01});
            TYPE_CURVE.style = VisStyle(style[0], style[1], style[2], style[3], style[4]);
            nh.param("STYLE_MESH", style, {0.357, 0.458, 0.710, 0.3, 0.005});
            TYPE_MESH.style = VisStyle(style[0], style[1], style[2], style[3], style[4]);
            nh.param("STYLE_FACET", style, {0.15, 0.6, 0.8, 0.1, 1.0});
            TYPE_FACET.style = VisStyle(style[0], style[1], style[2], style[3], style[4]);
            nh.param("STYLE_SCATTER", style, {1.0, 0.45, 0.0, 1.0, 0.02});
            TYPE_SCATTER.style = VisStyle(style[0], style[1], style[2], style[3], style[4]);
            nh.param("STYLE_SPHERE", style, {1.0, 0.45, 0.0, 1.0, 0.02});
            TYPE_SPHERE.style = VisStyle(style[0], style[1], style[2], style[3], style[4]);
            nh.param("STYLE_CUBE", style, {1.0, 0.45, 0.0, 1.0, 0.02});
            TYPE_CUBE.style = VisStyle(style[0], style[1], style[2], style[3], style[4]);
            nh.param("STYLE_ARROW", style, {1.0, 0.45, 0.0, 1.0, 0.02});
            TYPE_ARROW.style = VisStyle(style[0], style[1], style[2], style[3], style[4]);
            nh.param("STYLE_ARROW2", style, {0.8, 0.45, 0.8, 1.0, 0.04});
            TYPE_ARROW2.style = VisStyle(style[0], style[1], style[2], style[3], style[4]);
        };
    };

    class ROSVisualizer
    {
    private:
        std::string frame_id_ = "odom";
        std::string topic_name_ = "visualizer_markers";
        ROSVisualizerConfig config_;

        ros::NodeHandle nh_;
        ros::Publisher marker_pub_;
        visualization_msgs::MarkerArray marker_array_;

        // ID manager
        int group_shift_ = 8; // bit shift
        long long marker_group_ = 0;
        std::vector<std::pair<long long, long long>> marker_subid_list_; // Group | GroupSubID

    public:
        ROSVisualizer(ros::NodeHandle &nh);
        ROSVisualizer(ros::NodeHandle &nh, std::string frame_id, std::string topic_name);
        ~ROSVisualizer();

        long long getId(long long group_id, long long sub_id)
        {
            return group_id << group_shift_ | sub_id;
        };
        void setIdGroup(long long group_id)
        {
            if (group_id > -1)
                marker_group_ = group_id;
        };
        /**
         * @brief Update Id
         *
         * @param group_id Group ID to update (default: -1, not update group_id)
         * @return long long New ID
         */
        int32_t idUpdate(long long group_id = -1)
        {
            bool find_flag = false;
            if (group_id > -1)
                marker_group_ = group_id;
            for (uint i = 0; i < marker_subid_list_.size(); ++i)
            {
                if (marker_subid_list_[i].first == marker_group_)
                {
                    long long subid = marker_subid_list_[i].second + 1;
                    if (subid > (1 << (group_shift_ - 1)))
                        subid = 0;
                    marker_subid_list_[i] = std::make_pair(marker_group_, subid);
                    find_flag = true;
                    return getId(marker_group_, subid);
                }
            }
            if (!find_flag)
            {
                marker_subid_list_.push_back(std::make_pair(marker_group_, 0));
                return getId(marker_group_, 0);
            }
        };

        void resetId(void)
        {
            marker_group_ = 0;
            marker_subid_list_.clear();
        };

        void delGroup(long long group_id);
        void delType(const VisType &type);
        void delAll(void);

        void visArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const VisStyle &style);
        void visArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
        {
            visArrow(start, end, config_.TYPE_ARROW.style);
        }
        void delArrow(void);

        void visCurve(const std::vector<Eigen::Vector3d> &curve, const VisStyle &style);
        void visCurve(const std::vector<Eigen::Vector3d> &curve)
        {
            visCurve(curve, config_.TYPE_CURVE.style);
        }
        void delCurve(void);

        void visSphere(const Eigen::Vector3d &sphere, double radius, const VisStyle &style);
        void visSphere(const Eigen::Vector3d &sphere, double radius)
        {
            visSphere(sphere, radius, config_.TYPE_SPHERE.style);
        }
        void visSphere(const Eigen::Vector3d &sphere)
        {
            visSphere(sphere, config_.TYPE_SPHERE.style.x);
        }
        void visSphere(const std::vector<Eigen::Vector3d> &shperes, double radius, const VisStyle &style);
        void visSphere(const std::vector<Eigen::Vector3d> &shperes, double radius)
        {
            visSphere(shperes, radius, config_.TYPE_SPHERE.style);
        }
        void visSphere(const std::vector<Eigen::Vector3d> &spheres, const VisStyle &style)
        {
            visSphere(spheres, style.x, style);
        }
        void visSphere(const std::vector<Eigen::Vector3d> &shperes)
        {
            visSphere(shperes, config_.TYPE_SPHERE.style.x);
        }
        void delSphere(void);

        void visCube(const std::vector<Eigen::Vector3d> &cubes, const Eigen::Vector4d &quat, const VisStyle &style);
        void visCube(const std::vector<Eigen::Vector3d> &cubes, const Eigen::Vector4d &quat = Eigen::Vector4d(1, 0, 0, 0))
        {
            visCube(cubes, quat, config_.TYPE_CUBE.style);
        }
        void visCube(const Eigen::Vector3d &cube, const Eigen::Vector4d &quat, const VisStyle &style);
        void visCube(const Eigen::Vector3d &cube, const Eigen::Vector4d &quat = Eigen::Vector4d(1, 0, 0, 0))
        {
            visCube(cube, quat, config_.TYPE_CUBE.style);
        }
        void delCube(void);

        /**
         * @brief Visualize facets
         *
         * @param facet 3 points to form a facet
         * @param style
         */
        void visFacet(const std::vector<Eigen::Vector3d> &facet, const VisStyle &style);
        void visFacet(const std::vector<Eigen::Vector3d> &facet)
        {
            visFacet(facet, config_.TYPE_FACET.style);
        }
        void visFacet(const Eigen::MatrixX3d &facet, const VisStyle &style);
        void visFacet(const Eigen::MatrixX3d &facet)
        {
            visFacet(facet, config_.TYPE_FACET.style);
        }
        void delFacet(void);

        /**
         * @brief Visualize mesh
         *
         * @param mesh 2 points to form a segment
         * @param style
         */
        void visMesh(const std::vector<Eigen::Vector3d> &mesh, const VisStyle &style);
        void visMesh(const std::vector<Eigen::Vector3d> &mesh)
        {
            visMesh(mesh, config_.TYPE_MESH.style);
        }
        void visMesh(const Eigen::MatrixX3d &mesh, const VisStyle &style);
        void visMesh(const Eigen::MatrixX3d &mesh)
        {
            visMesh(mesh, config_.TYPE_MESH.style);
        }
        void delMesh(void);

        void visTwist(const Eigen::Vector3d &pos, const Eigen::Vector3d &linear, const Eigen::Vector3d &angular,
                      const VisStyle &linear_style, const VisStyle &angular_style);
        void visTwist(const Eigen::Vector3d &pos, const Eigen::Vector3d &linear, const Eigen::Vector3d &angular)
        {
            visTwist(pos, linear, angular, config_.TYPE_ARROW.style, config_.TYPE_ARROW2.style);
        }
    };

} // namespace ros_visualizer