#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "gcs_traj_opt/geo_utils/trajectory.hpp"
#include "gcs_traj_opt/geo_utils/quickhull.hpp"
#include "gcs_traj_opt/geo_utils/geo_utils.hpp"
#include "gcs_traj_opt/geo_utils/polyhedra.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define FRAME_ID "odom"

struct MarkerStyle
{
    double r, g, b, a;
    double x, y, z;
    MarkerStyle()
        : r(0.0), g(0.0), b(0.0), a(1.0), x(0.01), y(0.01), z(0.01){};
    MarkerStyle(double r_, double g_, double b_, double a_, double width_)
        : r(r_), g(g_), b(b_), a(a_), x(width_){};
    MarkerStyle(double r_, double g_, double b_, double a_, double x_, double y_, double z_)
        : r(r_), g(g_), b(b_), a(a_), x(x_), y(y_), z(z_){};
};

// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher meshPub;
    MarkerStyle meshStyle{0.0, 0.0, 1.0, 0.1, 1.0, 1.0, 1.0};
    ros::Publisher edgePub;
    MarkerStyle edgeStyle{0.247, 0.318, 0.710, 1, 0.005};
    ros::Publisher spherePub;
    ros::Publisher markersPub;

    visualization_msgs::Marker sphereMarkers;

    visualization_msgs::MarkerArray Markers;
    uint marker_id = 0;

public:
    ros::Publisher speedPub;
    ros::Publisher thrPub;
    ros::Publisher tiltPub;
    ros::Publisher bdrPub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);

        meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
        edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);

        spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);

        speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
        thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
        tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);
        // Add
        markersPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/markers", 1000);
    }

    // Visualize the trajectory and its front-end path
    template <int D>
    inline void visualize(const Trajectory<D> &traj,
                          const std::vector<Eigen::Vector3d> &route)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = FRAME_ID;
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = FRAME_ID;
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.30;

        if (route.size() > 0)
        {
            bool first = true;
            Eigen::Vector3d last;
            for (auto it : route)
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;
            }

            routePub.publish(routeMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = wps.col(i)(2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            double T = 0.01;
            Eigen::Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Eigen::Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub.publish(trajMarker);
        }
    }

    inline void visualizePolytope(const std::vector<Eigen::Matrix3Xd> &vPolys)
    {
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
        for (size_t id = 0; id < vPolys.size(); id++)
        {
            oldTris = mesh;

            quickhull::QuickHull<double> tinyQH;
            const auto polyHull = tinyQH.getConvexHull(vPolys[id].data(), vPolys[id].cols(), false, true);
            const auto &idxBuffer = polyHull.getIndexBuffer();
            int hNum = idxBuffer.size() / 3;

            curTris.resize(3, hNum * 3);
            for (int i = 0; i < hNum * 3; i++)
            {
                curTris.col(i) = vPolys[id].col(idxBuffer[i]);
            }
            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;
        }

        // RVIZ support tris for visualization
        visualization_msgs::Marker meshMarker, edgeMarker;

        meshMarker.id = 0;
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = FRAME_ID;
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.action = visualization_msgs::Marker::ADD;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";
        meshMarker.color.r = meshStyle.r;
        meshMarker.color.g = meshStyle.g;
        meshMarker.color.b = meshStyle.b;
        meshMarker.color.a = meshStyle.a;
        meshMarker.scale.x = meshStyle.x;
        meshMarker.scale.y = meshStyle.y;
        meshMarker.scale.z = meshStyle.z;

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = edgeStyle.r;
        edgeMarker.color.g = edgeStyle.g;
        edgeMarker.color.b = edgeStyle.b;
        edgeMarker.color.a = edgeStyle.a;
        edgeMarker.scale.x = edgeStyle.x;

        geometry_msgs::Point point;

        int ptnum = mesh.cols();

        for (int i = 0; i < ptnum; i++)
        {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i);
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }

        meshPub.publish(meshMarker);
        edgePub.publish(edgeMarker);

        return;
    }

    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
    {
        std::vector<Eigen::Matrix3Xd> vPolys;

        for (uint id = 0; id < hPolys.size(); id++)
        {
            Eigen::Matrix3Xd vPoly;
            geo_utils::enumerateVs(hPolys[id], vPoly);
            vPolys.push_back(vPoly);
        }
        visualizePolytope(vPolys);
    }

    inline void visualizePolytope(std::vector<Polyhedra> &polys)
    {
        std::vector<Eigen::Matrix3Xd> vPolys;
        for (auto &poly : polys)
        {
            vPolys.push_back(poly.getVRep());
        }
        visualizePolytope(vPolys);
    }

    inline void visualizePolytope(const Eigen::MatrixX4d &hPoly)
    {
        std::vector<Eigen::MatrixX4d> hPolys;
        hPolys.push_back(hPoly);
        visualizePolytope(hPolys);
    }

    inline void visualizePolytope(const Eigen::Matrix3Xd &vPoly)
    {
        std::vector<Eigen::Matrix3Xd> vPolys;
        vPolys.push_back(vPoly);
        visualizePolytope(vPolys);
    }

    // Visualize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = FRAME_ID;
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 1.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub.publish(sphereMarkers);
    }

    inline void deleteSphere()
    {
        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = FRAME_ID;
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::DELETE;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 1.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = 0.1;
        sphereMarkers.scale.y = 0.1;
        sphereMarkers.scale.z = 0.1;

        spherePub.publish(sphereMarkers);
        sphereMarkers.points.clear();
    }

    // FIXME: Cant visualize multiple curves (all curves will be connected)
    inline void visualizeCurve(const std::vector<Eigen::Vector3d> &curve, const MarkerStyle style = MarkerStyle())
    {
        visualization_msgs::Marker curveMarker;
        if (curve.size() < 2)
        {
            printf("Warning: Curve size is less than 2\n");
            return;
        }

        curveMarker.id = marker_id;
        curveMarker.type = visualization_msgs::Marker::LINE_STRIP;
        curveMarker.header.stamp = ros::Time::now();
        curveMarker.header.frame_id = FRAME_ID;
        curveMarker.pose.orientation.w = 1.00;
        curveMarker.action = visualization_msgs::Marker::ADD;
        curveMarker.ns = "curve";
        curveMarker.color.r = style.r;
        curveMarker.color.g = style.g;
        curveMarker.color.b = style.b;
        curveMarker.color.a = style.a;
        curveMarker.scale.x = style.x;

        geometry_msgs::Point point;

        for (auto it : curve)
        {
            point.x = it(0);
            point.y = it(1);
            point.z = it(2);
            curveMarker.points.push_back(point);
        }

        Markers.markers.push_back(curveMarker);
        markersPub.publish(Markers);
        marker_id++;
    }

    inline void deleteCurve()
    {
        for (auto &marker : Markers.markers)
        {
            if (marker.ns == "curve")
            {
                marker.action = visualization_msgs::Marker::DELETE;
            }
        }

        markersPub.publish(Markers);

        for (int i = 0; i < Markers.markers.size(); i++)
        {
            if (Markers.markers[i].ns == "curve")
            {
                Markers.markers.erase(Markers.markers.begin() + i);
                i--;
            }
        }
    }
};

#endif