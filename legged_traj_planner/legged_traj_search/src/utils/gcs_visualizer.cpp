/**
 * @file gcs_visualizer.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_search/utils/gcs_visualizer.hpp"

GCSVisualizer::GCSVisualizer(ros::NodeHandle &nh, std::string frame_id, std::string topic_name) : ros_visualizer::ROSVisualizer(nh, frame_id, topic_name)
{
}

void GCSVisualizer::visPolytope(const Eigen::Matrix3Xd &vPoly)
{
    std::vector<Eigen::Matrix3Xd> vPolys;
    vPolys.push_back(vPoly);
    visPolytope(vPolys);
}

void GCSVisualizer::visPolytope(const std::vector<Eigen::Matrix3Xd> &vPolys)
{
    Eigen::Matrix3Xd facets(3, 0), curTris(3, 0), oldTris(3, 0);
    for (size_t id = 0; id < vPolys.size(); id++)
    {
        oldTris = facets;
        quickhull::QuickHull<double> tinyQH;
        const auto polyHull = tinyQH.getConvexHull(vPolys[id].data(), vPolys[id].cols(), false, true);
        const auto &idxBuffer = polyHull.getIndexBuffer();
        int hNum = idxBuffer.size() / 3;

        curTris.resize(3, hNum * 3);
        for (int i = 0; i < hNum * 3; i++)
        {
            curTris.col(i) = vPolys[id].col(idxBuffer[i]);
        }
        facets.resize(3, oldTris.cols() + curTris.cols());
        facets.leftCols(oldTris.cols()) = oldTris;
        facets.rightCols(curTris.cols()) = curTris;
    }

    int ptnum = facets.cols();
    Eigen::Matrix3Xd mesh(3, ptnum * 2);
    for (int i = 0; i < ptnum / 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            mesh.col(6 * i + j) = facets.col(3 * i + j);
            mesh.col(6 * i + 3 + j) = facets.col(3 * i + (j + 1) % 3);
        }
    }
    visFacet(facets.transpose());
    visMesh(mesh.transpose());
    return;
}

void GCSVisualizer::visPolytope(const Eigen::MatrixX4d &hPoly)
{
    std::vector<Eigen::MatrixX4d> hPolys;
    hPolys.push_back(hPoly);
    visPolytope(hPolys);
}

void GCSVisualizer::visPolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
{
    std::vector<Eigen::Matrix3Xd> vPolys;
    for (uint id = 0; id < hPolys.size(); id++)
    {
        Eigen::Matrix3Xd vPoly;
        geo_utils::enumerateVs(hPolys[id], vPoly);
        vPolys.push_back(vPoly);
    }
    visPolytope(vPolys);
}

void GCSVisualizer::visPolytope(std::vector<Polyhedra> &polys)
{
    std::vector<Eigen::Matrix3Xd> vPolys;
    for (auto &poly : polys)
    {
        vPolys.push_back(poly.getVRep());
    }
    visPolytope(vPolys);
}

void GCSVisualizer::visPolytope(Polyhedra &poly)
{
    visPolytope(poly.getVRep());
}

void GCSVisualizer::visPolytope(Polyhedra poly)
{
    visPolytope(poly.getVRep());
}