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

#include "legged_traj_search/poly_traj/poly_traj_search.hpp"

PolyTrajSearch::PolyTrajSearch(IntersectBorder &intersect_border,
                               const bool enable_benchmark)
    : poly_corridor_(intersect_border.getPolyCorridor()),
      map_(intersect_border.getBorderCheck().getMap()),
      border_check_(intersect_border.getBorderCheck()),
      intersect_border_(intersect_border),
      benchmark_("PolyTrajSearch", enable_benchmark)
{
}

PolyTrajSearch::PolyTrajSearch(PolyCorridor &poly_corridor,
                               const grid_map::GridMap &map,
                               const std::string ground_layer,
                               const std::string ceiling_layer,
                               const bool enable_ground,
                               const bool enable_ceiling,
                               const bool enable_benchmark) : poly_corridor_(poly_corridor),
                                                              map_(map),
                                                              border_check_(poly_corridor, map, ground_layer, ceiling_layer, enable_ground, enable_ceiling),
                                                              intersect_border_(poly_corridor_, border_check_),
                                                              benchmark_("PolyTrajSearch", enable_benchmark)
{
}

bool PolyTrajSearch::endpointValid(const Point3D &start, const Point3D &goal)
{
    Point start_2d = start.head(2);
    Point goal_2d = goal.head(2);
    return intersect_border_.checkPointProjectInPoly(start_2d, 0) &&
           intersect_border_.checkPointProjectInPoly(goal_2d, intersect_border_.getPolyCorridor().getPolySize() - 1);
}

bool PolyTrajSearch::reachable(const Point3D &start, const Point3D &goal)
{
    if (reachable_ != 0)
        return reachable_ == 1;

    benchmark_.reset();
    if (!endpointValid(start, goal))
    {
        std::cout << "Warning: endpointValid failed" << std::endl;
        return false;
    }

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
    double border_length = 0;
    for (uint i = 0; i < border_.size() - 1; i++)
    {
        auto delta = border_[i + 1] - border_[i];
        border_length += sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
    }
    std::string msg = "Intersect Border - Length: " + to_string(border_length);
    benchmark_.addCustomData(border_length);
    benchmark_.record(msg, RecordType::CRITICAL);

    // Concave Points
    findConcavePoint(border_, concave_pts_);
    benchmark_.record("Find Concave Points", RecordType::CRITICAL);

    // Visiblity Graph Init
    // FIXME: is this appropriate?
    GridPt start_grid = border_check_.getIndexRemap().pos2Grid(start.head(2));
    GridPt goal_grid = border_check_.getIndexRemap().pos2Grid(goal.head(2));

    // FIXME: needs to improve the performance
    vis_graph_ = VisibilityGraph(border_, concave_pts_, start_grid, goal_grid);
    benchmark_.record("Visibility Graph Init", RecordType::CRITICAL);

    for (uint i = 0; i < vis_graph_.size(); i++)
    {
        if (i != 1 && vis_graph_.isVisibile(1, i))
        {
            reachable_ = 1;
            return true;
        }
    }
    reachable_ = -1;
    benchmark_.end();
    return false;
}

bool PolyTrajSearch::insertVerticalKeyPoint(std::vector<Point3D> &path, const int samples,
                                            double key_point_criteria, double margin)
{
    int size = path.size();

    for (int i = size - 2; i >= 0; i--)
    {
        Point3D start = path[i];
        Point3D goal = path[i + 1];
        std::vector<Point3D> key_points;
        std::vector<Point3D> sample_points;

        for (int j = 0; j < samples; j++)
        {
            double t = (double)j / (samples - 1);
            Point3D pos = start + t * (goal - start);
            sample_points.emplace_back(pos);
        }
        for (int j = 1; j < samples - 1; j++)
        {
            double diffs[3];
            diffs[0] = border_check_.queryHeight(Eigen::Vector2d(sample_points[j - 1].head(2))) - sample_points[j - 1][2];
            diffs[1] = border_check_.queryHeight(Eigen::Vector2d(sample_points[j].head(2))) - sample_points[j][2];
            diffs[2] = border_check_.queryHeight(Eigen::Vector2d(sample_points[j + 1].head(2))) - sample_points[j + 1][2];
            if (diffs[1] > 0 &&
                (2 * diffs[1] - diffs[0] - diffs[2]) / (Eigen::Vector2d(sample_points[j + 1].head(2) - sample_points[j - 1].head(2))).norm() > key_point_criteria)
            {
                key_points.emplace_back(Eigen::Vector3d(sample_points[j][0], sample_points[j][1], sample_points[j][2] + diffs[1] + margin));
            }
        }
        // Remove the key points that are local minimum
        for (int j = key_points.size() - 1; j > 0; j--)
        {
            if (2 * key_points[j][2] < key_points[j - 1][2] + key_points[j + 1][2])
            {
                key_points.erase(key_points.begin() + j);
            }
        }

        if (key_points.size() > 0)
        {
            path.insert(path.begin() + i + 1, key_points.begin(), key_points.end());
        }
    }
    return true;
}

bool PolyTrajSearch::search(const Point3D &start, const Point3D &goal, std::vector<Point3D> &path)
{
    path.clear();
    if (reachable_ == -1 || (reachable_ == 0 && !reachable(start, goal)))
    {
        std::cout << "Info: No solution - not reachable" << std::endl;
        return false;
    }

    // A* Search
    bool ret = GCS_AStarSearch(vis_graph_, grid_traj_);
    benchmark_.record("A* Search", RecordType::CRITICAL);
    if (!ret)
    {
        std::cout << "Info: No solution - A* Search failed" << std::endl;
        return false;
    }

    for (uint i = 0; i < grid_traj_.size(); i++)
    {
        Eigen::Vector3d pos;
        Eigen::Vector2d posxy = border_check_.getIndexRemap().grid2Pos(grid_traj_.at(i));
        pos[2] = border_check_.queryHeight(grid_traj_.at(i));
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        path.emplace_back(pos);
    }
    // Replace the start and goal with the original start and goal
    path.front() = start;
    path.back() = goal;

    benchmark_.record("Insert Vertical Key Point", RecordType::CRITICAL);
    insertVerticalKeyPoint(path, 20, 0.2, 0.05);

    benchmark_.end();
    return true;
}

bool PolyTrajSearch::searchStringStraining(const Point3D &start, const Point3D &goal, std::vector<Point3D> &path)
{
    path.clear();

    benchmark_.reset();
    if (!endpointValid(start, goal))
    {
        std::cout << "Warning: endpointValid failed" << std::endl;
        return false;
    }

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
    double border_length = 0;
    for (uint i = 0; i < border_.size() - 1; i++)
    {
        auto delta = border_[i + 1] - border_[i];
        border_length += sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
    }
    std::string msg = "Intersect Border - Length: " + to_string(border_length);
    benchmark_.record(msg, RecordType::CRITICAL);

    // String Straining
    GridPt start_grid = border_check_.getIndexRemap().pos2Grid(start.head(2));
    GridPt goal_grid = border_check_.getIndexRemap().pos2Grid(goal.head(2));

    StringStrainingSearch sss(border_, start_grid, goal_grid);
    bool ret = sss.search(grid_traj_, 4);
    benchmark_.record("String Straining Search", RecordType::CRITICAL);
    if (!ret)
    {
        std::cout << "Info: No solution - String Straining Search failed" << std::endl;
        return false;
    }

    for (uint i = 0; i < grid_traj_.size(); i++)
    {
        Eigen::Vector3d pos;
        Eigen::Vector2d posxy = border_check_.getIndexRemap().grid2Pos(grid_traj_.at(i));
        pos[2] = border_check_.queryHeight(grid_traj_.at(i));
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        path.emplace_back(pos);
    }
    // Replace the start and goal with the original start and goal
    path.front() = start;
    path.back() = goal;

    benchmark_.end();
    return true;
}
