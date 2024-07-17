/**
 * @file ros_visualizer.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "ros_visualizer/ros_visualizer.hpp"

namespace ros_visualizer
{

    ROSVisualizer::ROSVisualizer(ros::NodeHandle &nh) : nh_(nh)
    {
        config_.loadParams(nh);
        frame_id_ = config_.frame_id;
        topic_name_ = config_.topic_name;
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_, 10);
    }

    ROSVisualizer::ROSVisualizer(ros::NodeHandle &nh, std::string frame_id, std::string topic_name) : nh_(nh), frame_id_(frame_id), topic_name_(topic_name)
    {
        config_.loadParams(nh);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_, 10);
    }

    ROSVisualizer::~ROSVisualizer()
    {
    }

    void ROSVisualizer::delGroup(long long group_id)
    {
        for (auto &marker : marker_array_.markers)
        {
            if (marker.id >> group_shift_ == group_id)
            {
                marker.action = visualization_msgs::Marker::DELETE;
            }
        }

        marker_pub_.publish(marker_array_);

        for (int i = 0; i < marker_array_.markers.size(); i++)
        {
            if (marker_array_.markers[i].id >> 32 == group_id)
            {
                marker_array_.markers.erase(marker_array_.markers.begin() + i);
                i--;
            }
        }
    }

    void ROSVisualizer::delType(const VisType &type)
    {
        for (auto &marker : marker_array_.markers)
        {
            if (marker.ns == type.name_space)
            {
                marker.action = visualization_msgs::Marker::DELETE;
            }
        }

        marker_pub_.publish(marker_array_);

        for (int i = 0; i < marker_array_.markers.size(); i++)
        {
            if (marker_array_.markers[i].ns == type.name_space)
            {
                marker_array_.markers.erase(marker_array_.markers.begin() + i);
                i--;
            }
        }
    }

    // FIXME: Sometimes it cant clear
    void ROSVisualizer::delAll(void)
    {
        for (auto &marker : marker_array_.markers)
        {
            marker.action = visualization_msgs::Marker::DELETE;
        }
        marker_pub_.publish(marker_array_);
        marker_array_.markers.clear();
        resetId();
    }

    void ROSVisualizer::visArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_ARROW.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_ARROW.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = style.x;
        marker.scale.y = style.y;
        marker.scale.z = style.z;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        // Populate marker points
        geometry_msgs::Point p1;
        p1.x = start.x();
        p1.y = start.y();
        p1.z = start.z();
        marker.points.push_back(p1);

        geometry_msgs::Point p2;
        p2.x = end.x();
        p2.y = end.y();
        p2.z = end.z();
        marker.points.push_back(p2);

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::delArrow()
    {
        delType(config_.TYPE_ARROW);
    }

    void ROSVisualizer::visCurve(const std::vector<Eigen::Vector3d> &curve, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_CURVE.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_CURVE.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = style.x;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        // Populate marker points
        for (const auto &point : curve)
        {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marker.points.push_back(p);
        }

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::delCurve()
    {
        delType(config_.TYPE_CURVE);
    }

    void ROSVisualizer::visSphere(const Eigen::Vector3d &sphere, double radius, const VisStyle &style)
    {
        std::vector<Eigen::Vector3d> spheres;
        spheres.push_back(sphere);
        visSphere(spheres, radius, style);
    }

    void ROSVisualizer::visSphere(const std::vector<Eigen::Vector3d> &spheres, double radius, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_SPHERE.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_SPHERE.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = radius;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        // Populate marker points
        for (const auto &sphere : spheres)
        {
            geometry_msgs::Point p;
            p.x = sphere.x();
            p.y = sphere.y();
            p.z = sphere.z();
            marker.points.push_back(p);
        }

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::delSphere()
    {
        delType(config_.TYPE_SPHERE);
    }

    // NOTE: for multiple cubes it might rotate the frame first and then draw cubes in the rotated frame
    void ROSVisualizer::visCube(const std::vector<Eigen::Vector3d> &cubes, const Eigen::Vector4d &quat, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_CUBE.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_CUBE.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = quat(0);
        marker.pose.orientation.x = quat(1);
        marker.pose.orientation.y = quat(2);
        marker.pose.orientation.z = quat(3);
        marker.scale.x = style.x;
        marker.scale.y = style.y;
        marker.scale.z = style.z;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        // Populate marker points
        for (const auto &cube : cubes)
        {
            geometry_msgs::Point p;
            p.x = cube.x();
            p.y = cube.y();
            p.z = cube.z();
            marker.points.push_back(p);
        }

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::visCube(const Eigen::Vector3d &cube, const Eigen::Vector4d &quat, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_CUBE.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_CUBE.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cube.x();
        marker.pose.position.y = cube.y();
        marker.pose.position.z = cube.z();
        marker.pose.orientation.w = quat(0);
        marker.pose.orientation.x = quat(1);
        marker.pose.orientation.y = quat(2);
        marker.pose.orientation.z = quat(3);
        marker.scale.x = style.x;
        marker.scale.y = style.y;
        marker.scale.z = style.z;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        marker.points.push_back(geometry_msgs::Point());

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::delCube()
    {
        delType(config_.TYPE_CUBE);
    }

    void ROSVisualizer::visFacet(const std::vector<Eigen::Vector3d> &facet, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_FACET.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_FACET.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = style.x;
        marker.scale.y = style.y;
        marker.scale.z = style.z;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        // Populate marker points
        for (const auto &point : facet)
        {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marker.points.push_back(p);
        }

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::visFacet(const Eigen::MatrixX3d &facet, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_FACET.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_FACET.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = style.x;
        marker.scale.y = style.y;
        marker.scale.z = style.z;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        // Populate marker points
        for (int i = 0; i < facet.rows(); i++)
        {
            geometry_msgs::Point p;
            p.x = facet(i, 0);
            p.y = facet(i, 1);
            p.z = facet(i, 2);
            marker.points.push_back(p);
        }

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::delFacet()
    {
        delType(config_.TYPE_FACET);
    }

    void ROSVisualizer::visMesh(const std::vector<Eigen::Vector3d> &mesh, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_MESH.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_MESH.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = style.x;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        // Populate marker points
        for (const auto &point : mesh)
        {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marker.points.push_back(p);
        }

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::visMesh(const Eigen::MatrixX3d &mesh, const VisStyle &style)
    {
        visualization_msgs::Marker marker;
        // Populate marker fields
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = config_.TYPE_MESH.name_space;
        marker.id = idUpdate(marker_group_);
        marker.type = config_.TYPE_MESH.marker_type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = style.x;
        marker.color.r = style.r;
        marker.color.g = style.g;
        marker.color.b = style.b;
        marker.color.a = style.a;

        // Populate marker points
        for (int i = 0; i < mesh.rows(); i++)
        {
            geometry_msgs::Point p;
            p.x = mesh(i, 0);
            p.y = mesh(i, 1);
            p.z = mesh(i, 2);
            marker.points.push_back(p);
        }

        marker_array_.markers.push_back(marker);
        marker_pub_.publish(marker_array_);
    }

    void ROSVisualizer::delMesh()
    {
        delType(config_.TYPE_MESH);
    }

    void ROSVisualizer::visTwist(const Eigen::Vector3d &pos,
                                 const Eigen::Vector3d &linear,
                                 const Eigen::Vector3d &angular,
                                 const VisStyle &linear_style,
                                 const VisStyle &angular_style)
    {
        visArrow(pos, pos + linear, linear_style);
        visArrow(pos, pos + angular, angular_style);
    }

} // namespace ros_visualizer
