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

PolyTrajSearch::PolyTrajSearch(PolyCorridor &poly_corridor,
                               const grid_map::GridMap &map,
                               const std::string ground_layer,
                               const std::string ceiling_layer,
                               const bool enable_ground,
                               const bool enable_ceiling,
                               const bool enable_benchmark) : map_(map),
                                                              index_remap_(map),
                                                              border_check_(std::make_shared<CorridorBorderCheck>(poly_corridor, map,
                                                                                                                  ground_layer, ceiling_layer,
                                                                                                                  enable_ground, enable_ceiling)),
                                                              intersect_border_(border_check_),
                                                              benchmark_("PolyTrajSearch", enable_benchmark)
{
}

PolyTrajSearch::PolyTrajSearch(PolyCorridor &poly_corridor,
                               const grid_map::GridMap &map,
                               const PolyTrajSearchConfig config) : map_(map),
                                                                    index_remap_(map),
                                                                    border_check_(std::make_shared<CorridorBorderCheck>(poly_corridor, map,
                                                                                                                        config.ground_layer, config.ceiling_layer,
                                                                                                                        config.enable_ground, config.enable_ceiling)),
                                                                    intersect_border_(border_check_),
                                                                    benchmark_("PolyTrajSearch", config.enable_benchmark) {}

PolyTrajSearch::PolyTrajSearch(std::shared_ptr<BorderCheckBase> border_check,
                               const grid_map::GridMap &map,
                               const PolyTrajSearchConfig config) : map_(map),
                                                                    index_remap_(map),
                                                                    border_check_(border_check),
                                                                    intersect_border_(border_check_),
                                                                    benchmark_("PolyTrajSearch", config.enable_benchmark) {}

bool PolyTrajSearch::endpointValid(const Point3D &start, const Point3D &goal)
{
    GridPt start_2d = index_remap_.pos2Grid(start.head(2));
    GridPt goal_2d = index_remap_.pos2Grid(goal.head(2));
    return border_check_->isStartValid(start_2d) &&
           border_check_->isGoalValid(goal_2d);
}

bool PolyTrajSearch::reachable(const Point3D &start, const Point3D &goal, bool update_border)
{
    // if (reachable_ != 0)
    //     return reachable_ == 1;

    benchmark_.reset();
    if (!endpointValid(start, goal))
    {
        // std::cout << "Warning: endpointValid failed" << std::endl;
        return false;
    }

    // Intersect Border
    if (update_border)
    {
        GridPt start_2d = index_remap_.pos2Grid(start.head(2));
        if (!intersect_border_.getIntersectBorder(start_2d, border_))
        {
            std::cout << "Warning: intersect_border_.getIntersectBorder failed" << std::endl;
            return false;
        }
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
    if (update_border)
    {
        findConcavePoint(border_, concave_pts_);
        benchmark_.record("Find Concave Points", RecordType::CRITICAL);
    }

    // Visiblity Graph Init
    // FIXME: is this appropriate?
    // GridPt start_grid = index_remap_.pos2Grid(start.head(2));
    // GridPt goal_grid = index_remap_.pos2Grid(goal.head(2));
    Point start_grid = index_remap_.pos2GridFloat(start.head(2));
    Point goal_grid = index_remap_.pos2GridFloat(goal.head(2));

    // FIXME: needs to improve the performance
    vis_graph_ = VisibilityGraph(border_, concave_pts_, start_grid, goal_grid);
    benchmark_.record("Visibility Graph Init", RecordType::CRITICAL);

    bool reachable_goal = false, reachable_start = false;
    for (uint i = 0; i < vis_graph_.size(); i++)
    {
        if (i != 1 && vis_graph_.isVisibile(1, i))
            reachable_goal = 1;
        if (i != 0 && vis_graph_.isVisibile(0, i))
            reachable_start = 1;
        if (reachable_start && reachable_goal)
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
            diffs[0] = border_check_->queryHeight(Eigen::Vector2d(sample_points[j - 1].head(2))) - sample_points[j - 1][2];
            diffs[1] = border_check_->queryHeight(Eigen::Vector2d(sample_points[j].head(2))) - sample_points[j][2];
            diffs[2] = border_check_->queryHeight(Eigen::Vector2d(sample_points[j + 1].head(2))) - sample_points[j + 1][2];
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

/**
 * @brief
 *
 * @param path
 * @param alpha between 0 and 1
 * @return true
 * @return false
 */
bool heightConvexRelax(std::vector<Point3D> &path, double alpha = 0.5)
{
    int size = path.size();
    for (int i = 1; i < size - 1; i++)
    {
        double height = path[i][2];
        double height_prev = path[i - 1][2];
        double height_next = path[i + 1][2];
        double rate = (path[i] - path[i - 1]).head(2).norm() /
                      ((path[i + 1] - path[i]).head(2).norm() + (path[i] - path[i - 1]).head(2).norm());
        double kappa;
        if (height < (1 - rate) * height_prev + rate * height_next)
        {
            if (height_prev < height_next)
                kappa = (1 - rate) * alpha;
            else
                kappa = 1 - rate * alpha;
            path[i][2] = kappa * height_prev + (1 - kappa) * height_next;
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
        Eigen::Vector2d posxy = index_remap_.grid2Pos(grid_traj_.at(i));
        pos[2] = border_check_->queryHeight(grid_traj_.at(i)) + 0.01;
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        path.emplace_back(pos);
    }
    // Replace the start and goal with the original start and goal
    path.front() = start;
    path.back() = goal;

    benchmark_.record("Insert Vertical Key Point", RecordType::CRITICAL);
    insertVerticalKeyPoint(path, 20, 0.15, 0.01);
    heightConvexRelax(path, 0.5);
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
    GridPt start_2d = index_remap_.pos2Grid(start.head(2));
    GridPt goal_2d = index_remap_.pos2Grid(goal.head(2));
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
    GridPt start_grid = index_remap_.pos2Grid(start.head(2));
    GridPt goal_grid = index_remap_.pos2Grid(goal.head(2));

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
        Eigen::Vector2d posxy = index_remap_.grid2Pos(grid_traj_.at(i));
        pos[2] = border_check_->queryHeight(grid_traj_.at(i));
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

GridPolyLine PolyTrajSearch::getFullResBorder() const
{
    if (!border_.size())
        return GridPolyLine();
    GridPolyLine fullResBorder;
    fullResBorder.emplace_back(border_.front());
    for (uint i = 1; i < border_.size(); i++)
    {
        auto delta = border_[i] - border_[i - 1];
        int num = std::max(abs(delta[0]), abs(delta[1]));
        for (int j = 1; j <= num; j++)
        {
            fullResBorder.emplace_back(border_[i - 1] + j * delta / num);
        }
    }
    return fullResBorder;
}

// FeasiblePolyTrajSearch

FeasiblePolyTrajSearch::FeasiblePolyTrajSearch(std::shared_ptr<BorderCheckBase> border_check,
                                               const grid_map::GridMap &map,
                                               const bool enable_benchmark) : map_(map),
                                                                              index_remap_(map),
                                                                              border_check_(border_check),
                                                                              intersect_border_(border_check_),
                                                                              benchmark_("FeasiblePolyTrajSearch", enable_benchmark) {}
