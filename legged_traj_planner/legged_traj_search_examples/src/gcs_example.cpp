/**
 * @file gcs_example.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */

/* related header files */
#include "legged_traj_search_examples/gcs_example.hpp"
/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

/* internal project header files */
#include "legged_traj_search/poly_traj/gcs_astar_search.hpp"
#include "legged_traj_search/geo_utils/geo_utils_2d.hpp"
#include "legged_traj_search/poly_traj/intersect_border.hpp"
#include "legged_traj_search/geo_utils/guide_surf.hpp"
#include "legged_traj_search/poly_traj/minco_traj_init.hpp"

using namespace geo_utils_2d;

// Constructor & Destructor

GCS_Example::GCS_Example(GCS_Example_Config &conf,
                         ros::NodeHandle &nh_) : nh_(nh_), gcs_visualizer_(nh_), conf_(conf)
{
    ROS_INFO("GCS_Example");
    map_sub_ = nh_.subscribe(conf_.mapTopic, 1, &GCS_Example::map_callback, this);
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("gcs_example_mappub", 1, true);
    f = boost::bind(&GCS_Example::dyn_reconf_callback, this, _1, _2);
    server.setCallback(f);

    pos_shift = Eigen::MatrixX3d::Zero(1, 3);

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
        while (!map_received_ && ros::ok())
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("Map received!");
    }
}

// Callbacks

void GCS_Example::dyn_reconf_callback(legged_traj_search_examples::GCSExampleConfig &config, uint32_t level)
{
    start[0] = config.start_x;
    start[1] = config.start_y;
    pos_shift << config.pos_shift_x, config.pos_shift_y, config.pos_shift_z;
}

void GCS_Example::map_callback(const grid_map_msgs::GridMap::ConstPtr &msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    map_received_ = true;
    return;
}

// Utils

Eigen::Vector2d GCS_Example::getPos(const GridPt &idx)
{
    Eigen::Vector2d posxy;
    map_.getPosition(idx, posxy);
    return posxy;
}

Eigen::Matrix3Xd randomPoly(int samples = 20, double scale = 1.0)
{
    Eigen::Matrix3Xd mesh;
    Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vertices;
    Eigen::Vector3d inner;

    // Randomly generate a set of points
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5, 0.5);
    vertices.resize(3, samples);
    for (int i = 0; i < samples; i++)
    {
        vertices.col(i) << dis(gen), dis(gen), dis(gen);
    }
    vertices.array() *= scale;
    vertices.row(0).array() += dis(gen) * 2.0 * scale;
    vertices.row(1).array() += dis(gen) * 2.0 * scale;
    vertices.row(2).array() += dis(gen) * 2.0 * scale;

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

    return mesh;
}

Point3D randomPoint(double scale = 1.0)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5, 0.5);
    Point3D pt;
    pt << dis(gen) * scale, dis(gen) * scale, dis(gen) * scale;
    return pt;
}

bool GCS_Example::gcs_path_search(std::vector<Polyhedra> polys, Point3D start3d, Point3D goal3d,
                                  bool use_string_straining = false)
{
    // Init
    PolyCorridor poly_corridor(polys, start3d, goal3d);
    PolyTrajSearchConfig config;
    config.enable_benchmark = true;
    PolyTrajSearch poly_traj_search(poly_corridor, map_, config);
    std::vector<Point3D> path;

    // Check validity
    if (!poly_traj_search.endpointValid(start3d, goal3d))
        return false;
    poly_traj_search.reachable(start3d, goal3d);

    // Draw Start & Goal
    Point start = start3d.head(2);
    Point goal = goal3d.head(2);
    GridPt start_idx, goal_idx;
    start_idx = poly_traj_search.getIndexRemap().pos2Grid(start);
    goal_idx = poly_traj_search.getIndexRemap().pos2Grid(goal);
    // map_.getIndex(start, start_idx);
    // map_.getIndex(goal, goal_idx);
    Point3D start3d_grid, goal3d_grid;
    // start3d_grid.head(2) = getPos(start_idx);
    start3d_grid.head(2) = poly_traj_search.getIndexRemap().grid2Pos(start_idx);
    start3d_grid[2] = poly_traj_search.getBorderCheck()->queryHeight(start_idx);
    // goal3d_grid.head(2) = getPos(goal_idx);
    goal3d_grid.head(2) = poly_traj_search.getIndexRemap().grid2Pos(goal_idx);
    goal3d_grid[2] = poly_traj_search.getBorderCheck()->queryHeight(goal_idx);
    gcs_visualizer_.visSphere(start3d_grid, 0.01);
    gcs_visualizer_.visSphere(goal3d_grid, 0.01);
    gcs_visualizer_.visSphere(start3d, 0.02);
    gcs_visualizer_.visSphere(goal3d, 0.02);

    // Draw Corridor
    std::vector<Polyhedra> corridor = poly_corridor.getCorridor();
    gcs_visualizer_.visPolytope(corridor);
    // Draw Polys
    // gcs_visualizer_.visPolytope(polys);

    // Draw border
    GridPolyLine border = poly_traj_search.getBorder();
    std::vector<Point3D> border_pos;
    for (uint i = 0; i < border.size(); i++)
    {
        Point3D pos;
        Eigen::Vector2d posxy;
        pos[2] = poly_traj_search.getBorderCheck()->queryHeight(border.at(i));
        map_.getPosition(border.at(i), posxy);
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        border_pos.push_back(pos);
    }
    gcs_visualizer_.visCurve(border_pos);
    std::vector<Point3D> head_tail = {border_pos.front(), border_pos.back()};
    gcs_visualizer_.visCurve(head_tail, ros_visualizer::VisStyle(1.0, 0.3, 0.2, 0.5, 0.02));

    // Draw Concave Points
    std::vector<Point3D> concave_pts;
    for (auto pt : poly_traj_search.getConcavePts())
    {
        Point3D pos;
        // pos.head(2) = getPos(pt);
        pos.head(2) = poly_traj_search.getIndexRemap().grid2Pos(pt);
        pos[2] = poly_traj_search.getBorderCheck()->queryHeight(pt);
        concave_pts.push_back(pos);
    }
    gcs_visualizer_.visSphere(concave_pts, 0.03);
    if (use_string_straining)
        poly_traj_search.searchStringStraining(start3d, goal3d, path);
    if (use_string_straining && 0 ||
        !poly_traj_search.reachable(start3d, goal3d))
    {
        if (!use_string_straining)
            std::cout << "Warning: Goal is not reachable" << std::endl;
        else
            std::cout << "Warning: String Straining Search failed" << std::endl;
        return false;
    }
    else
    {
        // Path Search
        if (!use_string_straining && !poly_traj_search.search(start3d, goal3d, path))
        {
            std::cout << "Warning: A star search failed" << std::endl;
            return false;
        }
        else
        {
            // Draw Result
            // Draw VisGraph
            if (!use_string_straining)
            {
                VisibilityGraph vis_graph = poly_traj_search.getVisGraph();
                std::vector<Point3D> mesh;
                uint size = vis_graph.size();
                Point3D pos1, pos2;
                for (uint i = 0; i < size; i++)
                {
                    for (uint j = i + 1; j < size; j++)
                    {
                        if (vis_graph.isVisibile(i, j))
                        {
                            // pos1.head(2) = getPos(vis_graph.getPt(i));
                            // pos2.head(2) = getPos(vis_graph.getPt(j));
                            pos1.head(2) = poly_traj_search.getIndexRemap().grid2Pos(vis_graph.getPt(i));
                            pos2.head(2) = poly_traj_search.getIndexRemap().grid2Pos(vis_graph.getPt(j));
                            pos1[2] = poly_traj_search.getBorderCheck()->queryHeight(vis_graph.getPt(i));
                            pos2[2] = poly_traj_search.getBorderCheck()->queryHeight(vis_graph.getPt(j));
                            mesh.push_back(pos1);
                            mesh.push_back(pos2);
                        }
                    }
                }
                gcs_visualizer_.visMesh(mesh, ros_visualizer::VisStyle(0.3, 0.3, 0.3, 0.3, 0.01));
            }
            // Draw grid_traj
            gcs_visualizer_.visCurve(path, ros_visualizer::VisStyle(1.0, 0.3, 0.2, 1.0, 0.02));

            // Minco Traj Opt
            MincoTrajInit minco_traj_opt(path);
            std::vector<Point3D> traj;
            bool ret = minco_traj_opt.getTrajSamples(traj, 0.01);
            if (!ret)
            {
                std::cout << "Warning: Minco Traj Opt failed" << std::endl;
                return true;
            }
            gcs_visualizer_.visCurve(traj, ros_visualizer::VisStyle(0.3, 0.8, 0.3, 1.0, 0.02));

            return true;
        }
    }
}

/**
 * @brief gcs_path_search Performance test
 *
 * @param polys
 * @param start3d
 * @param goal3d
 * @param result Benchmark Result
 * @param records Benchmark Records
 * @param use_string_straining
 * @return int 0: Success
 *             1: Endpoint not valid
 *             2: Goal not reachable
 *             3: A star search failed
 *             4: Minco Traj Opt failed
 */
int GCS_Example::gcs_path_search_perf(std::vector<Polyhedra> polys, Point3D start3d, Point3D goal3d,
                                      BenchmarkResult &result, std::vector<Record> &records,
                                      bool use_string_straining = false)
{
    // Init
    PolyCorridor poly_corridor(polys, start3d, goal3d);
    PolyTrajSearchConfig cfg;
    cfg.enable_benchmark = true;
    PolyTrajSearch poly_traj_search(poly_corridor, map_, cfg);
    std::vector<Point3D> path;

    // Check validity
    if (!poly_traj_search.endpointValid(start3d, goal3d))
        return 1;
    poly_traj_search.reachable(start3d, goal3d);

    // Draw Start & Goal
    Point start = start3d.head(2);
    Point goal = goal3d.head(2);
    GridPt start_idx, goal_idx;
    start_idx = poly_traj_search.getIndexRemap().pos2Grid(start);
    goal_idx = poly_traj_search.getIndexRemap().pos2Grid(goal);
    // map_.getIndex(start, start_idx);
    // map_.getIndex(goal, goal_idx);
    Point3D start3d_grid, goal3d_grid;
    // start3d_grid.head(2) = getPos(start_idx);
    start3d_grid.head(2) = poly_traj_search.getIndexRemap().grid2Pos(start_idx);
    start3d_grid[2] = poly_traj_search.getBorderCheck()->queryHeight(start_idx);
    // goal3d_grid.head(2) = getPos(goal_idx);
    goal3d_grid.head(2) = poly_traj_search.getIndexRemap().grid2Pos(goal_idx);
    goal3d_grid[2] = poly_traj_search.getBorderCheck()->queryHeight(goal_idx);
    gcs_visualizer_.visSphere(start3d_grid, 0.01);
    gcs_visualizer_.visSphere(goal3d_grid, 0.01);
    gcs_visualizer_.visSphere(start3d, 0.02);
    gcs_visualizer_.visSphere(goal3d, 0.02);

    // Draw Corridor
    std::vector<Polyhedra> corridor = poly_corridor.getCorridor();
    gcs_visualizer_.visPolytope(corridor);
    // Draw Polys
    // gcs_visualizer_.visPolytope(polys);

    // Draw border
    GridPolyLine border = poly_traj_search.getBorder();
    std::vector<Point3D> border_pos;
    for (uint i = 0; i < border.size(); i++)
    {
        Point3D pos;
        Eigen::Vector2d posxy;
        pos[2] = poly_traj_search.getBorderCheck()->queryHeight(border.at(i));
        map_.getPosition(border.at(i), posxy);
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        border_pos.push_back(pos);
    }
    gcs_visualizer_.visCurve(border_pos);
    std::vector<Point3D> head_tail = {border_pos.front(), border_pos.back()};
    gcs_visualizer_.visCurve(head_tail, ros_visualizer::VisStyle(1.0, 0.3, 0.2, 0.5, 0.02));

    // Draw Concave Points
    std::vector<Point3D> concave_pts;
    for (auto pt : poly_traj_search.getConcavePts())
    {
        Point3D pos;
        // pos.head(2) = getPos(pt);
        pos.head(2) = poly_traj_search.getIndexRemap().grid2Pos(pt);
        pos[2] = poly_traj_search.getBorderCheck()->queryHeight(pt);
        concave_pts.push_back(pos);
    }
    gcs_visualizer_.visSphere(concave_pts, 0.03);
    if (use_string_straining)
        poly_traj_search.searchStringStraining(start3d, goal3d, path);
    if (use_string_straining && 0 ||
        !poly_traj_search.reachable(start3d, goal3d))
    {
        if (!use_string_straining)
            std::cout << "Warning: Goal is not reachable" << std::endl;
        else
            std::cout << "Warning: String Straining Search failed" << std::endl;
        return 2;
    }
    else
    {
        // Path Search
        if (!use_string_straining && !poly_traj_search.search(start3d, goal3d, path))
        {
            std::cout << "Warning: A star search failed" << std::endl;
            return 3;
        }
        else
        {
            // Draw Result
            // Draw VisGraph
            if (!use_string_straining)
            {
                VisibilityGraph vis_graph = poly_traj_search.getVisGraph();
                std::vector<Point3D> mesh;
                uint size = vis_graph.size();
                Point3D pos1, pos2;
                for (uint i = 0; i < size; i++)
                {
                    for (uint j = i + 1; j < size; j++)
                    {
                        if (vis_graph.isVisibile(i, j))
                        {
                            // pos1.head(2) = getPos(vis_graph.getPt(i));
                            // pos2.head(2) = getPos(vis_graph.getPt(j));
                            pos1.head(2) = poly_traj_search.getIndexRemap().grid2Pos(vis_graph.getPt(i));
                            pos2.head(2) = poly_traj_search.getIndexRemap().grid2Pos(vis_graph.getPt(j));
                            pos1[2] = poly_traj_search.getBorderCheck()->queryHeight(vis_graph.getPt(i));
                            pos2[2] = poly_traj_search.getBorderCheck()->queryHeight(vis_graph.getPt(j));
                            mesh.push_back(pos1);
                            mesh.push_back(pos2);
                        }
                    }
                }
                gcs_visualizer_.visMesh(mesh, ros_visualizer::VisStyle(0.3, 0.3, 0.3, 0.3, 0.01));
            }
            // Draw grid_traj
            gcs_visualizer_.visCurve(path, ros_visualizer::VisStyle(1.0, 0.3, 0.2, 1.0, 0.02));

            // Minco Traj Opt
            MincoTrajInit minco_traj_opt(path);
            std::vector<Point3D> traj;
            bool ret = minco_traj_opt.getTrajSamples(traj, 0.01);
            if (!ret)
            {
                std::cout << "Warning: Minco Traj Opt failed" << std::endl;
                return 4;
            }
            gcs_visualizer_.visCurve(traj, ros_visualizer::VisStyle(0.3, 0.8, 0.3, 1.0, 0.02));
            result = poly_traj_search.getResult();
            records = poly_traj_search.getRecords();
            return 0;
        }
    }
}
// Examples

bool GCS_Example::example_run(std::string name)
{
    if (name == "eg_guide_surface_demo")
    {
        eg_guide_surface();
    }
    else if (name == "eg_convoluted_guide_surface_demo")
    {
        eg_convoluted_guide_surface();
    }
    else if (name == "eg_keypoint_guide_surface_demo")
    {
        eg_keypoint_guide_surface();
    }
    else if (name == "eg_gcs_barrier_demo")
    {
        eg_gcs_barrier_demo();
    }
    else if (name == "eg_gcs_barrier_ani_demo")
    {
        eg_gcs_barrier_ani_demo();
    }
    else if (name == "eg_gcs_rand_corridor_demo")
    {
        eg_gcs_rand_corridor_demo();
    }
    else if (name == "eg_gcs_rand_map_demo")
    {
        eg_gcs_rand_map_demo();
    }
    else if (name == "perf_gcs_rand_corridor_demo")
    {
        perf_gcs_rand_corridor_demo();
        return false;
    }
    else
    {
        std::cout << "Example not found" << std::endl;
    }
    return true;
}

void GCS_Example::eg_guide_surface()
{
    int poly_num = conf_.polyNum;

    gcs_visualizer_.delAll();
    std::vector<Polyhedra> polys;

    for (int i = 0; i < poly_num; i++)
    {
        Point3D randPos = randomPoint(2.0);
        randPos[2] *= 1.5;
        Eigen::Matrix3Xd tmp1 = randomPoly(20, 0.6);
        Eigen::Matrix3Xd tmp2 = (tmp1.array().colwise() + (randPos.array() + pos_shift.transpose().col(0).array())).eval();
        polys.emplace_back(Polyhedra(tmp2));
    }
    PolyCorridor corridor(polys);

    std::vector<Point3D> key_points;
    for (auto poly : corridor.getPolys())
    {
        key_points.push_back(poly.getInterior());
    }

    gcs_visualizer_.visPolytope(corridor.getCorridor());
    HarmonicGuideSurf guide_surf(key_points, 1);
    map_.add("guide_surf");
    Eigen::Vector3d key_points_mean = Eigen::Vector3d::Zero();
    for (auto pt : key_points)
    {
        key_points_mean += pt;
    }
    key_points_mean /= key_points.size();
    map_.setPosition(key_points_mean.head(2));
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Position pos;
        map_.getPosition(*iterator, pos);
        map_.at("guide_surf", *iterator) = guide_surf.getHeight(pos);
    }
    grid_map_msgs::GridMap gm_message;
    grid_map::GridMapRosConverter::toMessage(map_, gm_message);
    map_pub_.publish(gm_message);

    std::shared_ptr<CorridorBorderCheck> border_check = std::make_shared<CorridorBorderCheck>(corridor, map_, "elevation", "ceiling", false, false);
    IndexRemap index_remap(map_);
    IntersectBorder intersect_border(border_check);
    GridPt start_2d = index_remap.pos2Grid(key_points.front().head(2));
    GridPt goal_2d = index_remap.pos2Grid(key_points.back().head(2));
    GridPolyLine border;
    bool ret = intersect_border.getIntersectBorder(start_2d, goal_2d, border);
    if (ret)
    {
        std::vector<Point3D> border_pos;
        for (uint i = 0; i < border.size(); i++)
        {
            Point3D pos;
            Eigen::Vector2d posxy;
            pos[2] = border_check->queryHeight(border.at(i));
            posxy = index_remap.grid2Pos(border.at(i));
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            border_pos.emplace_back(pos);
        }
        border_pos.emplace_back(border_pos.front());
        gcs_visualizer_.visCurve(border_pos, ros_visualizer::VisStyle(1.0, 0.6, 0.002, 1.0, 0.03));
    }
    else
    {
        std::cout << "Get intersecting border failed" << std::endl;
    }

    std::cout << "Press any key to continue..." << std::endl;
    getchar();
    return;
}

void GCS_Example::eg_convoluted_guide_surface()
{
    gcs_visualizer_.delAll();

    ConvolutedGuideSurf guide_surf(map_, conf_.kernel_size, conf_.kernel_interval, "elevation");
    map_.add("guide_surf");
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Position pos;
        map_.getPosition(*iterator, pos);
        map_.at("guide_surf", *iterator) = guide_surf.getHeight(pos);
    }
    grid_map_msgs::GridMap gm_message;
    grid_map::GridMapRosConverter::toMessage(map_, gm_message);
    map_pub_.publish(gm_message);

    std::cout << "Press any key to continue..." << std::endl;
    getchar();
    return;
}

void GCS_Example::eg_keypoint_guide_surface()
{
    gcs_visualizer_.delAll();

    Eigen::Vector3d p0(-0.7, 0, 0.0), p1(0.7, 0, 0.0);
    // Guide Surf
    int samples = 8;
    Eigen::Vector3d pmid = (p0 + p1) / 2;
    // Max
    for (int i = 1; i < samples + 1; i++)
    {
        pmid[2] = std::max(pmid[2], (double)map_.atPosition("elevation",
                                                            (p0 + (p1 - p0) * i / (samples + 1)).head(2)));
    }
    std::vector<Point3D> key_points = {p0, pmid, p1};

    HarmonicGuideSurf guide_surf(key_points);
    gcs_visualizer_.visSphere(key_points, 0.1);
    map_.add("guide_surf");
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Position pos;
        map_.getPosition(*iterator, pos);
        map_.at("guide_surf", *iterator) = guide_surf.getHeight(pos);
    }
    grid_map_msgs::GridMap gm_message;
    grid_map::GridMapRosConverter::toMessage(map_, gm_message);
    map_pub_.publish(gm_message);

    std::cout << "Press any key to continue..." << std::endl;
    getchar();
    return;
}

void GCS_Example::eg_gcs_barrier_demo()
{
    // clean
    gcs_visualizer_.delAll();

    Eigen::MatrixX3d waypoints(3, 3);
    waypoints << -0.5, 0.0, 0.2,
        0.0, 0.4, 0.4,
        // -0.2, 0.0, 0.5,
        // 0.2, 0.0, 0.5,
        // 0.3, 0.0, 0.3,
        0.5, 0.0, 0.2;
    // Eigen::Vector2d start(-0.4, -0.3), goal(0.4, -0.3);
    Eigen::Vector2d goal(0.4, -0.3);

    Eigen::Vector3d start3d, goal3d;
    start3d << start[0], start[1], map_.atPosition("elevation", start);
    goal3d << goal[0], goal[1], map_.atPosition("elevation", goal);
    gcs_visualizer_.visSphere(start3d, 0.01);
    gcs_visualizer_.visSphere(goal3d, 0.01);

    std::vector<Polyhedra> polys;
    for (int i = 0; i < waypoints.rows(); i++)
    {
        Eigen::Matrix3Xd tmpvPoly = (vPoly.array().colwise() + (waypoints.transpose().col(i).array() + pos_shift.transpose().col(0).array())).eval();
        polys.emplace_back(Polyhedra(tmpvPoly));
    }

    gcs_path_search(polys, start3d, goal3d);
}

void GCS_Example::eg_gcs_barrier_ani_demo()
{
    double sleep_time = 1.0;
    double short_sleep_time = 0.1;
    // clean
    gcs_visualizer_.delAll();

    Eigen::MatrixX3d waypoints(3, 3);
    waypoints << -0.5, 0.0, 0.2,
        0.0, 0.4, 0.4,
        // -0.2, 0.0, 0.5,
        // 0.2, 0.0, 0.5,
        // 0.3, 0.0, 0.3,
        0.5, 0.0, 0.2;
    // Eigen::Vector2d start(-0.4, -0.3), goal(0.4, -0.3);
    Eigen::Vector2d goal(0.4, -0.3);

    Eigen::Vector3d start3d, goal3d;
    start3d << start[0], start[1], map_.atPosition("elevation", start);
    goal3d << goal[0], goal[1], map_.atPosition("elevation", goal);

    ros::Duration(sleep_time).sleep();

    std::vector<Polyhedra> polys;
    for (int i = 0; i < waypoints.rows(); i++)
    {
        Eigen::Matrix3Xd tmpvPoly = (vPoly.array().colwise() + (waypoints.transpose().col(i).array() + pos_shift.transpose().col(0).array())).eval();
        polys.emplace_back(Polyhedra(tmpvPoly));
    }

    // Init
    PolyCorridor poly_corridor(polys, start3d, goal3d);
    PolyTrajSearchConfig config;
    config.enable_benchmark = true;
    PolyTrajSearch poly_traj_search(poly_corridor, map_, config);
    std::vector<Point3D> path;

    // Check validity
    if (!poly_traj_search.endpointValid(start3d, goal3d))
        return;
    poly_traj_search.reachable(start3d, goal3d);

    ros::Duration(sleep_time).sleep();

    // Draw Start & Goal
    Point start2d = start3d.head(2);
    Point goal2d = goal3d.head(2);
    GridPt start_idx, goal_idx;
    start_idx = poly_traj_search.getIndexRemap().pos2Grid(start2d);
    goal_idx = poly_traj_search.getIndexRemap().pos2Grid(goal2d);
    // map_.getIndex(start, start_idx);
    // map_.getIndex(goal, goal_idx);
    Point3D start3d_grid, goal3d_grid;
    // start3d_grid.head(2) = getPos(start_idx);
    start3d_grid.head(2) = poly_traj_search.getIndexRemap().grid2Pos(start_idx);
    start3d_grid[2] = poly_traj_search.getBorderCheck()->queryHeight(start_idx);
    // goal3d_grid.head(2) = getPos(goal_idx);
    goal3d_grid.head(2) = poly_traj_search.getIndexRemap().grid2Pos(goal_idx);
    goal3d_grid[2] = poly_traj_search.getBorderCheck()->queryHeight(goal_idx);
    // gcs_visualizer_.visSphere(start3d_grid, 0.01);
    // gcs_visualizer_.visSphere(goal3d_grid, 0.01);
    gcs_visualizer_.visSphere(start3d, 0.04, ros_visualizer::VisStyle(0.0, 0.6, 1.0, 1.0, 0.04));
    ros::Duration(short_sleep_time).sleep();
    gcs_visualizer_.visSphere(goal3d, 0.04);

    ros::Duration(sleep_time).sleep();

    // Draw Corridor
    std::vector<Polyhedra> corridor = poly_corridor.getCorridor();
    gcs_visualizer_.visPolytope(corridor);
    // Draw Polys
    // gcs_visualizer_.visPolytope(polys);

    ros::Duration(sleep_time).sleep();

    // Draw border
    GridPolyLine border = poly_traj_search.getFullResBorder();
    std::vector<Point3D> border_pos;
    for (uint i = 0; i < border.size(); i++)
    {
        Point3D pos;
        Eigen::Vector2d posxy;
        pos[2] = poly_traj_search.getBorderCheck()->queryHeight(border.at(i));
        map_.getPosition(border.at(i), posxy);
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        border_pos.push_back(pos);
    }
    for (int i = 0; i < border_pos.size(); i++)
    {
        std::vector<Point3D> segment = {border_pos.at(i), border_pos.at((i + 1) % border_pos.size())};
        gcs_visualizer_.visCurve(segment, ros_visualizer::VisStyle(0.0, 0.0, 0.0, 0.5, 0.015));
        ros::Duration(short_sleep_time).sleep();
    }

    ros::Duration(sleep_time).sleep();

    // Draw Concave Points
    std::vector<Point3D> concave_pts;
    for (auto pt : poly_traj_search.getConcavePts())
    {
        Point3D pos;
        // pos.head(2) = getPos(pt);
        pos.head(2) = poly_traj_search.getIndexRemap().grid2Pos(pt);
        pos[2] = poly_traj_search.getBorderCheck()->queryHeight(pt);
        concave_pts.push_back(pos);
        gcs_visualizer_.visSphere(pos, 0.03);
        ros::Duration(short_sleep_time).sleep();
    }

    ros::Duration(sleep_time).sleep();

    if (!poly_traj_search.reachable(start3d, goal3d))
    {
        std::cout << "Warning: String Straining Search failed" << std::endl;
        return;
    }
    else
    {
        // Path Search
        if (!poly_traj_search.search(start3d, goal3d, path))
        {
            std::cout << "Warning: A star search failed" << std::endl;
            return;
        }
        else
        {
            // Draw Result
            // Draw VisGraph

            VisibilityGraph vis_graph = poly_traj_search.getVisGraph();
            std::vector<Point3D> mesh;
            uint size = vis_graph.size();
            Point3D pos1, pos2;
            for (uint i = 0; i < size; i++)
            {
                for (uint j = i + 1; j < size; j++)
                {
                    if (vis_graph.isVisibile(i, j))
                    {
                        // pos1.head(2) = getPos(vis_graph.getPt(i));
                        // pos2.head(2) = getPos(vis_graph.getPt(j));
                        pos1.head(2) = poly_traj_search.getIndexRemap().grid2Pos(vis_graph.getPt(i));
                        pos2.head(2) = poly_traj_search.getIndexRemap().grid2Pos(vis_graph.getPt(j));
                        pos1[2] = poly_traj_search.getBorderCheck()->queryHeight(vis_graph.getPt(i));
                        pos2[2] = poly_traj_search.getBorderCheck()->queryHeight(vis_graph.getPt(j));
                        mesh.push_back(pos1);
                        mesh.push_back(pos2);
                    }
                }
                gcs_visualizer_.visMesh(mesh, ros_visualizer::VisStyle(0.0, 0.0, 0.0, 0.4, 0.005));
                ros::Duration(short_sleep_time*3).sleep();
                mesh.clear();
            }

            ros::Duration(sleep_time).sleep();

            // Draw grid_traj
            gcs_visualizer_.visCurve(path, ros_visualizer::VisStyle(1.0, 0.6, 0.002, 1.0, 0.015));

            ros::Duration(sleep_time).sleep();

            // Minco Traj Opt
            MincoTrajInit minco_traj_opt(path);
            std::vector<Point3D> traj;
            bool ret = minco_traj_opt.getTrajSamples(traj, 0.01);
            if (!ret)
            {
                std::cout << "Warning: Minco Traj Opt failed" << std::endl;
                return;
            }
            // gcs_visualizer_.visCurve(traj, ros_visualizer::VisStyle(1.0, 0.3, 0.3, 0.4, 0.02));

            ros::Duration(sleep_time).sleep();

            return;
        }
    }
}

void GCS_Example::eg_gcs_rand_corridor_demo()
{
    // Settings
    int poly_num = conf_.polyNum;
    int samples = 20;
    double poly_scale = 0.4;
    double poly_pos_scale_xy = 1.5;
    double poly_pos_scale_z = 0.5;

    // Wait for the user to press a key
    std::cout << "Press any key to continue..." << std::endl;
    getchar();

    std::vector<Polyhedra> polys;
    Point3D start, goal;
    do
    {
        gcs_visualizer_.delAll();
        polys.clear();
        for (int i = 0; i < poly_num; i++)
        {
            Eigen::Matrix3Xd tmp1 = randomPoly(samples, poly_scale);
            Point3D randPt = randomPoint();
            randPt.head(2) *= poly_pos_scale_xy;
            randPt[2] *= poly_pos_scale_z;
            Eigen::Matrix3Xd tmp2 = (tmp1.array().colwise() + (randPt.array() + pos_shift.transpose().col(0).array())).eval();
            polys.emplace_back(Polyhedra(tmp2));
        }
        start = polys.at(0).getInterior();
        goal = polys.at(poly_num - 1).getInterior();
    } while (!gcs_path_search(polys, start, goal, false));
}

void GCS_Example::eg_gcs_rand_map_demo()
{
    Eigen::MatrixX3d poly(10, 3);
    poly << -0.3, -0.3, -0.3,
        0.23, -0.3, -0.3,
        0.3, 0.25, -0.3,
        -0.2, 0.3, -0.3,
        -0.3, -0.3, 0.3,
        0.3, -0.3, 0.3,
        0.3, 0.3, 0.3,
        -0.3, 0.3, 0.3,
        0.0, 0.1, 0.44,
        -0.4, 0.2, 0.4;
    Eigen::MatrixX3d waypoints(6, 3);
    waypoints << 0.0, 0.0, 0.8,
        0.0, -1.0, 0.7,
        1.0, -1.0, 0.4,
        1.0, 1.0, 0.7,
        -1.0, 1.0, 0.5,
        -1.0, -1.0, 0.8;

    std::vector<Polyhedra> polys;
    for (int i = 0; i < waypoints.rows(); i++)
    {
        Eigen::Matrix3Xd tmpvPoly = (poly.transpose().array().colwise() + (waypoints.transpose().col(i).array() + pos_shift.transpose().col(0).array())).eval();
        polys.emplace_back(Polyhedra(tmpvPoly));
    }

    Point3D start3d(0.0, 0.0, 0.8);
    Point3D goal3d(-1.0, -1.0, 0.8);
    gcs_visualizer_.delAll();
    gcs_path_search(polys, start3d, goal3d, false);
}

void GCS_Example::perf_gcs_rand_corridor_demo()
{
    std::vector<std::vector<Record>> records_list;
    std::vector<BenchmarkResult> result_list;

    std::vector<Record> records;
    BenchmarkResult result;

    int endpoint_invalid = 0;
    int unreachable = 0;
    int astar_fail = 0;

    // Settings
    int try_num = 1000;
    int cnt = 0;

    // Gen Settings
    int poly_num = conf_.polyNum;
    int samples = 20;
    double poly_scale = 0.4;
    double poly_pos_scale_xy = 1.5;
    double poly_pos_scale_z = 0.5;

    while (cnt < try_num)
    {
        gcs_visualizer_.delAll();
        std::vector<Polyhedra> polys;
        Point3D start, goal;

        polys.clear();
        for (int i = 0; i < poly_num; i++)
        {
            Eigen::Matrix3Xd tmp1 = randomPoly(samples, poly_scale);
            Point3D randPt = randomPoint();
            randPt.head(2) *= poly_pos_scale_xy;
            randPt[2] *= poly_pos_scale_z;
            Eigen::Matrix3Xd tmp2 = (tmp1.array().colwise() + (randPt.array() + pos_shift.transpose().col(0).array())).eval();
            polys.emplace_back(Polyhedra(tmp2));
        }
        start = polys.at(0).getInterior();
        goal = polys.at(poly_num - 1).getInterior();

        int ret = gcs_path_search_perf(polys, start, goal, result, records, false);
        if (ret == 0)
        {
            cnt++;
            records_list.push_back(records);
            result_list.push_back(result);
        }
        else if (ret == 1)
            endpoint_invalid++;
        else if (ret == 2)
            unreachable++;
        else if (ret == 3)
            astar_fail++;
    }

    // Output
    std::string data_file_path = conf_.data_file_path;
    std::ofstream data_file(data_file_path);
    if (!data_file.is_open())
    {
        std::cout << "Error: Open file failed" << std::endl;
        return;
    }
    data_file << "normalTime, criticalTime, miscTime, totTime, borderSize";
    for (auto record : records_list.at(0))
    {
        data_file << ", " << record.name;
    }
    data_file << std::endl;
    for (int i = 0; i < result_list.size(); i++)
    {
        data_file << result_list.at(i).normal_tot_time << ", " << result_list.at(i).critic_tot_time << ", "
                  << result_list.at(i).misc_tot_time << ", " << result_list.at(i).tot_time << ", " << result_list.at(i).custom_data[0];
        for (auto record : records_list.at(i))
        {
            data_file << ", " << record.time_record;
        }
        data_file << std::endl;
    }
}