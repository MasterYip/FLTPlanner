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
#include <memory>
/* external project header files */

/* internal project header files */
#include "legged_traj_search/poly_traj/gcs_astar_search.hpp"
#include "legged_traj_search/poly_traj/intersect_border.hpp"
#include "legged_traj_search/poly_traj/string_straining.hpp"
#include "legged_traj_search/geo_utils/geo_utils_2d.hpp"
#include "legged_traj_search/geo_utils/guide_surf.hpp"
#include "legged_traj_search/utils/benchmark.hpp"

// class PolyTrajSearchBase
// {
// protected:
//     std::shared_ptr<grid_map::GridMap> map_;
//     std::shared_ptr<IndexRemap> index_remap_;
//     std::shared_ptr<BorderCheckBase> border_check_;
//     std::shared_ptr<IntersectBorder> intersect_border_;
//     std::shared_ptr<VisibilityGraph> vis_graph_;
//     Benchmark benchmark_;

// };

struct PolyTrajSearchConfig
{
    std::string ground_layer = "elevation";
    std::string ceiling_layer = "ceiling";
    bool enable_ground = true;
    bool enable_ceiling = false;

    bool enable_benchmark = false;
};

class PolyTrajSearch
{
private:
    // input data
    const grid_map::GridMap map_;
    IndexRemap index_remap_;
    CorridorBorderCheck border_check_;
    IntersectBorder intersect_border_;
    VisibilityGraph vis_graph_;

    Benchmark benchmark_;
    int reachable_ = 0; // 0: unknown, 1: reachable, -1: unreachable

    // output data
    GridPolyLine border_;
    GridPoints concave_pts_;
    GridPolyLine grid_traj_;

public:
    PolyTrajSearch(PolyCorridor &poly_corridor,
                   const grid_map::GridMap &map,
                   const std::string ground_layer,
                   const std::string ceiling_layer,
                   const bool enable_ground,
                   const bool enable_ceiling,
                   const bool enable_benchmark = false);
    PolyTrajSearch(PolyCorridor &poly_corridor,
                   const grid_map::GridMap &map,
                   const PolyTrajSearchConfig config = PolyTrajSearchConfig());
    ~PolyTrajSearch() = default;
    // Poly Traj Search
    bool endpointValid(const Point3D &start, const Point3D &goal);
    bool reachable(const Point3D &start, const Point3D &goal);
    bool search(const Point3D &start, const Point3D &goal, std::vector<Point3D> &path);
    bool searchStringStraining(const Point3D &start, const Point3D &goal, std::vector<Point3D> &path);
    // Traj Init
    bool insertVerticalKeyPoint(std::vector<Point3D> &path, const int samples = 20,
                                double key_point_criteria = 0.2,
                                double margin = 0.05);
    // Getters
    // Objects
    CorridorBorderCheck &getBorderCheck() { return border_check_; }
    IndexRemap &getIndexRemap() { return index_remap_; }

    // Data
    GridPolyLine getBorder() const { return border_; }
    GridPoints getConcavePts() const { return concave_pts_; }
    GridPolyLine getGridTraj() const { return grid_traj_; }
    VisibilityGraph getVisGraph() const { return vis_graph_; }

    // Benchmark
    std::vector<Record> getRecords() { return benchmark_.getRecords(); }
    BenchmarkResult getResult() { return benchmark_.getResult(); }
};