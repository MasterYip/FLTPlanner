#include "polyve/polyve.h"

#include <cstdlib>
#include <climits>
#include <random>

using namespace std;
using namespace ros;
using namespace Eigen;

PolyVe::PolyVe(Config &conf, NodeHandle &nh_)
    : config(conf), nh(nh_), visualization(config, nh), visualizer(nh)
{
    triggerSub = nh.subscribe(config.triggerTopic, 1, &PolyVe::triggerCallBack,
                              this, TransportHints().tcpNoDelay());
}

PolyVe::~PolyVe() {}

void PolyVe::triggerCallBack(const std_msgs::Empty::ConstPtr &msg)
{
    conductVE();
    return;
}

const Eigen::Matrix3Xd PolyVe::genVpoly()
{

    // ---------------------------- Test Data Generation ----------------------------

    Eigen::Matrix3Xd new_vert, recoveredV;
    Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vertices;
    Eigen::Vector3d inner;

    // Randomly generate a set of points
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5, 0.5);
    vertices.resize(3, config.randomTries);
    for (int i = 0; i < config.randomTries; i++)
    {
        vertices.col(i) << dis(gen), dis(gen), dis(gen);
    }
    vertices.array() *= config.randomScale;
    vertices.row(0).array() += dis(gen) * 2.0 * config.randomScale;
    vertices.row(1).array() += dis(gen) * 2.0 * config.randomScale;
    vertices.row(2).array() += dis(gen) * 2.0 * config.randomScale;

    // Get the convex hull(that includes the points) in mesh
    quickhull::QuickHull<double> qh;
    const auto cvxHull = qh.getConvexHull(vertices.data(),
                                          vertices.cols(),
                                          false, false);
    const auto &vtBuffer = cvxHull.getVertexBuffer();
    int vts = vtBuffer.size();
    new_vert.resize(3, vts);
    quickhull::Vector3<double> v;
    for (int i = 0; i < vts; i++)
    {
        v = vtBuffer[i];
        new_vert(0, i) = v.x;
        new_vert(1, i) = v.y;
        new_vert(2, i) = v.z;
    }
    return new_vert;
}

void PolyVe::addVpoly(const Eigen::Matrix3Xd &vPoly)
{
    vPolyBuf_.push_back(vPoly);
    visualizer.visualizePolytope(vPolyBuf_);
    return;
}

void PolyVe::addVpoly()
{
    Eigen::Matrix3Xd recoveredV = genVpoly();
    addVpoly(recoveredV);
    return;
}

////////////////////
// Tests
////////////////////

/**
 * @brief Original test function
 */
void PolyVe::conductVE(void)
{
    std::cout << "------------------------------------" << std::endl;

    // ---------------------------- Test Data Generation ----------------------------

    Eigen::Matrix3Xd mesh, recoveredV;
    Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vertices;
    Eigen::Vector3d inner;

    // Randomly generate a set of points
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5, 0.5);
    vertices.resize(3, config.randomTries);
    for (int i = 0; i < config.randomTries; i++)
    {
        vertices.col(i) << dis(gen), dis(gen), dis(gen);
    }
    vertices.array() *= config.randomScale;
    vertices.row(0).array() += dis(gen) * 2.0 * config.randomScale;
    vertices.row(1).array() += dis(gen) * 2.0 * config.randomScale;
    vertices.row(2).array() += dis(gen) * 2.0 * config.randomScale;

    // Get the convex hull(that includes the points) in mesh
    quickhull::QuickHull<double> qh;
    const auto cvxHull = qh.getConvexHull(vertices.data(),
                                          vertices.cols(),
                                          false, false);
    const auto &idBuffer = cvxHull.getIndexBuffer(); // A buffer storing order of indices for triangle vertices revisiting
    const auto &vtBuffer = cvxHull.getVertexBuffer();
    int ids = idBuffer.size();
    // The mesh is represented by triangles(3 vertices) in right hand rule USING idBuffer
    mesh.resize(3, ids);
    quickhull::Vector3<double> v;
    for (int i = 0; i < ids; i++)
    {
        v = vtBuffer[idBuffer[i]];
        mesh(0, i) = v.x;
        mesh(1, i) = v.y;
        mesh(2, i) = v.z;
    }

    // Obtain the half space intersection(H-rep) form from the mesh
    Eigen::MatrixX4d hPoly(ids / 3, 4);
    Eigen::Vector3d normal, point, edge0, edge1;
    for (int i = 0; i < ids / 3; i++)
    {
        point = mesh.col(3 * i + 1);
        edge0 = point - mesh.col(3 * i);
        edge1 = mesh.col(3 * i + 2) - point;
        normal = edge0.cross(edge1).normalized();
        hPoly(i, 0) = normal(0);
        hPoly(i, 1) = normal(1);
        hPoly(i, 2) = normal(2);
        hPoly(i, 3) = -normal.dot(point);
    }

    // Generate some redundant half spaces
    Eigen::Array4d hParamRange;
    hParamRange.head<3>() = hPoly.leftCols<3>().cwiseAbs().colwise().maxCoeff().transpose();
    hParamRange(3) = hPoly.rightCols<1>().cwiseAbs().maxCoeff();
    Eigen::Vector4d halfSpace;
    Eigen::MatrixX4d redundantHs(config.redundantTryH, 4);
    int validNum = 0;
    for (int i = 0; i < config.redundantTryH; i++)
    {
        halfSpace(0) = dis(gen);
        halfSpace(1) = dis(gen);
        halfSpace(2) = dis(gen);
        halfSpace(3) = dis(gen);
        halfSpace.array() *= 2.0 * hParamRange;
        if (halfSpace.head<3>().squaredNorm() > 0 &&
            (halfSpace.head<3>().transpose() * vertices).maxCoeff() < halfSpace(3))
        {
            redundantHs(validNum, 0) = halfSpace(0);
            redundantHs(validNum, 1) = halfSpace(1);
            redundantHs(validNum, 2) = halfSpace(2);
            redundantHs(validNum, 3) = -halfSpace(3);
            validNum++;
        }
    }
    std::cout << "Number of Redundant Half Space: " << validNum << std::endl;
    Eigen::MatrixX4d mergedHs(validNum + hPoly.rows(), 4);
    mergedHs.topRows(hPoly.rows()) = hPoly;
    mergedHs.bottomRows(validNum) = redundantHs.topRows(validNum);

    // ---------------------------- Test Vertex Enumeration ----------------------------

    if (!geo_utils::enumerateVs(mergedHs, recoveredV))
    {
        std::cout << "Vertex Enumeration Fails Once !!!" << std::endl;
        return;
    }

    // ---------------------------- Visualize All Results ----------------------------
    std::cout << "Number of Enumerated Vertices: " << recoveredV.cols() << std::endl;
    geo_utils::findInterior(hPoly, inner); // recalculate iterior just for visualization
    visualization.visualizeVertices(recoveredV);
    visualization.visualizeInterior(inner);
    visualization.visualizeMesh(mesh);

    return;
}

// Pass
void PolyVe::vPolyMergeTest(void)
{
    Eigen::Matrix3Xd vp1, vp2, mesh;
    std::vector<Eigen::Matrix3Xd> vPolyBuf;
    vPolyBuf.push_back(genVpoly());
    vPolyBuf.push_back(genVpoly());
    vPolyBuf.push_back(geo_utils::mergeVpoly(vPolyBuf[0], vPolyBuf[1]));
    visualizer.visualizePolytope(vPolyBuf);
}

// Pass
void PolyVe::vPoly2hPolyTest(void)
{
    Eigen::Matrix3Xd vPoly = genVpoly();
    Eigen::Matrix3Xd vPoly2;
    Eigen::MatrixX4d hPoly = geo_utils::vpoly2hpoly(vPoly);
    std::vector<Eigen::Matrix3Xd> vPolyBuf;

    vPolyBuf.push_back(vPoly);
    visualizer.visualizePolytope(vPolyBuf);
    geo_utils::enumerateVs(hPoly, vPoly2);
    visualization.visualizeVertices(vPoly2);
}

// Pass
void PolyVe::inHpolyTest(void)
{
    Eigen::Matrix3Xd vPoly = genVpoly();
    Eigen::MatrixX4d hPoly = geo_utils::vpoly2hpoly(vPoly);
    // Eigen::Vector3d inner;
    // geo_utils::findInterior(hPoly, inner);
    // visualization.visualizeInterior(inner);
    for (int i = 0; i < vPoly.cols(); i++)
    {
        if (!geo_utils::inHpoly(hPoly, vPoly.col(i), 1e-6))
        {
            std::cout << "inHpolyTest Fails !!!" << std::endl;
            return;
        }
    }
    std::cout << "inHpolyTest Passes !!!" << std::endl;
}

// Pass
void PolyVe::vPolyIntersectTest(void)
{
    std::vector<Eigen::Matrix3Xd> vPolyBuf;
    Eigen::Matrix3Xd vPolyIntersect, vp1, vp2;
    bool ret = false;
    while (!ret)
    {
        vp1 = genVpoly();
        vp2 = genVpoly();
        ret = geo_utils::intersectVpoly(vp1, vp2, vPolyIntersect);
    }
    vPolyBuf.push_back(vp1);
    vPolyBuf.push_back(vp2);
    vPolyBuf.push_back(vPolyIntersect);
    visualizer.visualizePolytope(vPolyBuf);
    visualization.visualizeVertices(vPolyIntersect);
}

void PolyVe::CorridorTest(void)
{
    Eigen::MatrixX3d FootHull(10, 3);
    Eigen::Matrix3Xd vPoly(3, 10);
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
    visualizer.visualizePolytope(CorridorBuf);
}

////////////////////
// Visualization
////////////////////

Visualization::Visualization(Config &conf, NodeHandle &nh_)
    : config(conf), nh(nh_)
{
    meshPub = nh.advertise<visualization_msgs::Marker>(config.meshTopic, 1000);
    edgePub = nh.advertise<visualization_msgs::Marker>(config.edgeTopic, 1000);
    verticesPub = nh.advertise<visualization_msgs::Marker>(config.vertexTopic, 1000);
    interiorPub = nh.advertise<visualization_msgs::Marker>(config.interiorTopic, 1000);
}

void Visualization::visualizeMesh(const Eigen::Matrix3Xd &mesh)
{
    visualization_msgs::Marker meshMarker, edgeMarker;

    // The mesh is represented by triangles(3 vertices) in right hand rule
    meshMarker.id = 0;
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = "map";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = "mesh";
    meshMarker.color.r = 0.00;
    meshMarker.color.g = 0.00;
    meshMarker.color.b = 1.00;
    meshMarker.color.a = 0.5;
    meshMarker.scale.x = 1.00;
    meshMarker.scale.y = 1.00;
    meshMarker.scale.z = 1.00;

    // The edge is represented by lines(2 vertices)
    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 0.00;
    edgeMarker.color.g = 1.00;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 1.00;
    edgeMarker.scale.x = 0.005 * config.randomScale;

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
            // NOTE: Edges is duplicated
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

void Visualization::visualizeVertices(const Eigen::Matrix3Xd &vertices)
{
    visualization_msgs::Marker pointsMarker;

    pointsMarker.id = 0;
    pointsMarker.header.stamp = ros::Time::now();
    pointsMarker.header.frame_id = "map";
    pointsMarker.pose.orientation.w = 1.00;
    pointsMarker.action = visualization_msgs::Marker::ADD;
    pointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    pointsMarker.ns = "vertices";
    pointsMarker.color.r = 1.00;
    pointsMarker.color.g = 0.00;
    pointsMarker.color.b = 0.00;
    pointsMarker.color.a = 1.00;
    pointsMarker.scale.x = 0.02 * config.randomScale;
    pointsMarker.scale.y = pointsMarker.scale.x;
    pointsMarker.scale.z = pointsMarker.scale.x;

    for (int i = 0; i < vertices.cols(); i++)
    {
        geometry_msgs::Point point;
        point.x = vertices.col(i)(0);
        point.y = vertices.col(i)(1);
        point.z = vertices.col(i)(2);
        pointsMarker.points.push_back(point);
    }

    verticesPub.publish(pointsMarker);

    return;
}

void Visualization::visualizeInterior(const Eigen::Vector3d &interior)
{
    visualization_msgs::Marker pointsMarker;

    pointsMarker.id = 0;
    pointsMarker.header.stamp = ros::Time::now();
    pointsMarker.header.frame_id = "map";
    pointsMarker.pose.orientation.w = 1.00;
    pointsMarker.action = visualization_msgs::Marker::ADD;
    pointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    pointsMarker.ns = "interior";
    pointsMarker.color.r = 1.00;
    pointsMarker.color.g = 1.00;
    pointsMarker.color.b = 1.00;
    pointsMarker.color.a = 1.00;
    pointsMarker.scale.x = 0.02 * config.randomScale;
    pointsMarker.scale.y = pointsMarker.scale.x;
    pointsMarker.scale.z = pointsMarker.scale.x;

    geometry_msgs::Point point;
    point.x = interior(0);
    point.y = interior(1);
    point.z = interior(2);
    pointsMarker.points.push_back(point);

    interiorPub.publish(pointsMarker);

    return;
}
