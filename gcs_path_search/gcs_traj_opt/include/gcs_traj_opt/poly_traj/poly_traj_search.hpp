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
#include "gcs_traj_opt/poly_traj/gcs_astar_search.hpp"
#include "gcs_traj_opt/poly_traj/intersect_border.hpp"
#include "gcs_traj_opt/geo_utils/geo_utils_2d.hpp"
#include "gcs_traj_opt/geo_utils/guide_surf.hpp"
#include "gcs_traj_opt/utils/benchmark.hpp"

class PolyTrajSearch
{
private:
    // input data
    IntersectBorder &intersect_border_;
    PolyCorridor &poly_corridor_;
    BorderCheck &border_check_;
    const grid_map::GridMap &map_;
    VisibilityGraph vis_graph_;

    Benchmark benchmark_;
    // output data
    GridPolyLine border_;
    GridPoints concave_pts_;
    GridPolyLine grid_traj_;

public:
    PolyTrajSearch(IntersectBorder &intersect_border);
    ~PolyTrajSearch() = default;

    bool search(const Point3D &start, const Point3D &goal, std::vector<Point3D> &path);

    GridPolyLine getBorder() const { return border_; }
    GridPoints getConcavePts() const { return concave_pts_; }
    GridPolyLine getGridTraj() const { return grid_traj_; }
    VisibilityGraph getVisGraph() const { return vis_graph_; }
};