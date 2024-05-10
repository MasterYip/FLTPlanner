/**
 * @file poly_traj_search.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
#include "legged_traj_search/poly_traj/gcs_astar_search.hpp"
#include "legged_traj_search/poly_traj/intersect_border.hpp"
#include "legged_traj_search/poly_traj/string_straining.hpp"
#include "legged_traj_search/geo_utils/geo_utils_2d.hpp"
#include "legged_traj_search/geo_utils/guide_surf.hpp"
#include "legged_traj_search/utils/benchmark.hpp"

class PolyTrajSearch
{
private:
    // input data
    PolyCorridor poly_corridor_;
    const grid_map::GridMap map_;
    BorderCheck border_check_;
    IntersectBorder intersect_border_;
    VisibilityGraph vis_graph_;

    Benchmark benchmark_;
    int reachable_ = 0; // 0: unknown, 1: reachable, -1: unreachable
    // output data
    GridPolyLine border_;
    GridPoints concave_pts_;
    GridPolyLine grid_traj_;

public:
    PolyTrajSearch(IntersectBorder &intersect_border, const bool enable_benchmark = false);
    PolyTrajSearch(PolyCorridor &poly_corridor,
                   const grid_map::GridMap &map,
                   const std::string ground_layer = "elevation",
                   const std::string ceiling_layer = "ceiling",
                   const bool enable_ground = true,
                   const bool enable_ceiling = false,
                   const bool enable_benchmark = false);
    ~PolyTrajSearch() = default;
    // Poly Traj Search
    bool endpointValid(const Point3D &start, const Point3D &goal);
    bool reachable(const Point3D &start, const Point3D &goal);
    bool search(const Point3D &start, const Point3D &goal, std::vector<Point3D> &path);
    bool searchStringStraining(const Point3D &start, const Point3D &goal, std::vector<Point3D> &path);

    // Getters
    // Data
    GridPolyLine getBorder() const { return border_; }
    GridPoints getConcavePts() const { return concave_pts_; }
    GridPolyLine getGridTraj() const { return grid_traj_; }
    VisibilityGraph getVisGraph() const { return vis_graph_; }
    // Objects
    BorderCheck &getBorderCheck() { return border_check_; }

    // Benchmark
    std::vector<Record> getRecords() { return benchmark_.getRecords(); }
    BenchmarkResult getResult() { return benchmark_.getResult(); }
};