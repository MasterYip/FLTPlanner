/**
 * @file poly_traj_search.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "gcs_traj_opt/poly_traj/poly_traj_search.hpp"

PolyTrajSearch::PolyTrajSearch(IntersectBorder &intersect_border)
    : intersect_border_(intersect_border),
      poly_corridor_(intersect_border.getPolyCorridor()),
      border_check_(intersect_border.getBorderCheck()),
      map_(border_check_.getMap()), benchmark_("PolyTrajSearch")
{
}

bool PolyTrajSearch::search(const Point3D &start, const Point3D &goal, std::vector<Point3D> &path)
{
    benchmark_.reset();
    std::vector<Point3D> init_path;

    // Intersect Border
    Point start_2d = start.head(2);
    Point goal_2d = goal.head(2);
    if (!intersect_border_.getIntersectBorder(start_2d, goal_2d, border_))
    {
        std::cout << "Warning: intersect_border_.getIntersectBorder failed" << std::endl;
        return false;
    }
    if (border_.size() < 3)
    {
        std::cout << "Warning: border_.size() < 3" << std::endl;
        return false;
    }
    benchmark_.record("Intersect Border", RecordType::CRITICAL);

    // Concave Points
    findConcavePoint(border_, concave_pts_);
    benchmark_.record("Find Concave Points", RecordType::CRITICAL);

    // Visiblity Graph Init
    GridPt start_grid, goal_grid; // FIXME: is this appropriate?
    map_.getIndex(start.head(2), start_grid);
    map_.getIndex(goal.head(2), goal_grid);
    // FIXME: needs to improve the performance
    vis_graph_ = VisibilityGraph(border_, concave_pts_, start_grid, goal_grid);
    benchmark_.record("Visibility Graph Init", RecordType::CRITICAL);

    // A* Search
    bool ret = GCS_AStarSearch(vis_graph_, grid_traj_);
    benchmark_.record("A* Search", RecordType::CRITICAL);
    if (!ret)
    {
        std::cout << "Info: No solution - A* Search failed" << std::endl;
        return false;
    }

    benchmark_.end();
    // TODO: path init
    return ret;
}