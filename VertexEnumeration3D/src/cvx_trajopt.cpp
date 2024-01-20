
/* related header files */
#include "cvx_trajopt/cvx_trajopt.h"
/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include <grid_map_ros/grid_map_ros.hpp>
/* internal project header files */
#include "geo_utils/geo_utils_2d.hpp"

using namespace geo_utils_2d;

class GridPtState : public AStarState<GridPtState>
{
private:
    uint pt_idx_;
    VisibilityGraph *vis_graph_;

public:
    GridPtState() : pt_idx_(0), vis_graph_(nullptr){};
    GridPtState(uint pt_idx, VisibilityGraph *vis_graph) : pt_idx_(pt_idx), vis_graph_(vis_graph){};
    uint getPtIdx()
    {
        return pt_idx_;
    };
    float GoalDistanceEstimate(GridPtState &nodeGoal) override
    {
        return (vis_graph_->getPt(pt_idx_) - vis_graph_->getPt(nodeGoal.getPtIdx())).matrix().norm();
    }; // Heuristic function which computes the estimated cost to the goal node
    bool IsGoal(GridPtState &nodeGoal) override
    {
        return nodeGoal.getPtIdx() == pt_idx_;
    }; // Returns true if this node is the goal node
    bool GetSuccessors(AStarSearch<GridPtState> *astarsearch, GridPtState *parent_node) override
    {
        GridPtState newnode;
        for (uint i = 0; i < vis_graph_->size(); i++)
        {
            if ((parent_node && i != parent_node->getPtIdx() && i != pt_idx_ && vis_graph_->isVisibile(pt_idx_, i)) ||
                (!parent_node && i != pt_idx_ && vis_graph_->isVisibile(pt_idx_, i)))
            {
                newnode = GridPtState(i, vis_graph_);
                astarsearch->AddSuccessor(newnode);
            }
        }
        return true;
    }; // Retrieves all successors to this node and adds them via astarsearch.addSuccessor()
    float GetCost(GridPtState &successor) override
    {
        return (vis_graph_->getPt(pt_idx_) - vis_graph_->getPt(successor.getPtIdx())).matrix().norm();
    }; // Computes the cost of travelling from this node to the successor node
    bool IsSameState(GridPtState &rhs) override
    {
        return rhs.getPtIdx() == pt_idx_;
    }; // Returns true if this node is the same as the rhs node
    size_t Hash()
    {
        return pt_idx_;
    }; // Returns a hash for the state
};

bool GCS_AStarSearch(VisibilityGraph &vis_graph, std::vector<GridPt> &path)
{
    // A* Search
    AStarSearch<GridPtState> astarsearch;

    GridPtState start_state(0, &vis_graph);
    GridPtState goal_state(1, &vis_graph);
    astarsearch.SetStartAndGoalStates(start_state, goal_state);
    uint SearchState;
    uint SearchSteps = 0;
    do
    {
        SearchState = astarsearch.SearchStep();
        SearchSteps++;
    } while (SearchState == AStarSearch<GridPtState>::SEARCH_STATE_SEARCHING);
    if (SearchState == AStarSearch<GridPtState>::SEARCH_STATE_SUCCEEDED)
    {
        GridPtState *node = astarsearch.GetSolutionStart();
        path.clear();
        path.emplace_back(vis_graph.getPt(node->getPtIdx()));
        while (true)
        {
            node = astarsearch.GetSolutionNext();
            if (!node)
                break;
            path.emplace_back(vis_graph.getPt(node->getPtIdx()));
        };
        astarsearch.FreeSolutionNodes();
        astarsearch.EnsureMemoryFreed();
        return true;
    }
    astarsearch.FreeSolutionNodes();
    // astarsearch.EnsureMemoryFreed();
    return false;
}

CVX_TrajOpt::CVX_TrajOpt(CVX_TrajOpt_Config &conf, ros::NodeHandle &nh_) : nh_(nh_), visualizer_(nh_), conf_(conf)
{
    ROS_INFO("CVX_TrajOpt::CVX_TrajOpt()");
    map_sub_ = nh_.subscribe(conf_.mapTopic, 1, &CVX_TrajOpt::map_callback, this);

    // 并将回调函数和服务端绑定
    f = boost::bind(&CVX_TrajOpt::dyn_reconf_callback, this, _1, _2);
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

CVX_TrajOpt::~CVX_TrajOpt()
{
}

void CVX_TrajOpt::dyn_reconf_callback(polyve::CvxTrajOptConfig &config, uint32_t level)
{
    start[0] = config.start_x;
    start[1] = config.start_y;
    pos_shift << config.pos_shift_x, config.pos_shift_y, config.pos_shift_z;
}

void CVX_TrajOpt::map_callback(const grid_map_msgs::GridMap::ConstPtr &msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    map_received_ = true;
    return;
}

void CVX_TrajOpt::drawSphereIdx(const GridPt &idx, const double radius = 0.01)
{
    Eigen::Vector3d pos;
    Eigen::Vector2d posxy;
    pos[2] = map_.at("elevation", idx);
    map_.getPosition(idx, posxy);
    pos[0] = posxy.x();
    pos[1] = posxy.y();
    visualizer_.visualizeSphere(pos, radius);
    return;
}

void CVX_TrajOpt::drawSegmentIdx(const GridPt &idx1, const GridPt &idx2)
{
    Eigen::Vector3d pos1, pos2;
    Eigen::Vector2d posxy1, posxy2;
    pos1[2] = map_.at("elevation", idx1);
    map_.getPosition(idx1, posxy1);
    pos1[0] = posxy1.x();
    pos1[1] = posxy1.y();
    pos2[2] = map_.at("elevation", idx2);
    map_.getPosition(idx2, posxy2);
    pos2[0] = posxy2.x();
    pos2[1] = posxy2.y();
    std::vector<Eigen::Vector3d> pts;
    pts.push_back(pos1);
    pts.push_back(pos2);
    visualizer_.visualizeCurve(pts, MarkerStyle(1, 1, 1, 0.1, 0.005));
    return;
}

////////////////////
// Corridor Intersect Border

bool inCorridor(const std::vector<Eigen::Matrix3Xd> &Corridor, const Eigen::Vector3d &pos)
{
    for (uint i = 0; i < Corridor.size(); i++)
    {
        if (geo_utils::inVpoly(Corridor.at(i), pos))
        {
            return true;
        }
    }
    return false;
}

bool inCorridor(const std::vector<Eigen::Matrix3Xd> &Corridor,
                const grid_map::GridMap &map,
                const GridPt &idx,
                const std::string maplayer = "elevation")
{
    Eigen::Vector3d pos;
    Eigen::Vector2d posxy;
    pos[2] = map.at(maplayer, idx);
    map.getPosition(idx, posxy);
    pos[0] = posxy.x();
    pos[1] = posxy.y();
    return inCorridor(Corridor, pos);
}

/**
 * @brief Get the Border of Corrider&GridMap intersection
 * FIXME: not stable
 * BUG: This method is not a complusory condition for probable solution
 * @param Corridor
 * @param start
 * @param goal
 * @param connectivity
 * @return std::vector<GridPt> `Clockwise` sequence of border points (z axis projected)
 */
std::vector<GridPt> CVX_TrajOpt::getCorriderIntersectBorder(const std::vector<Eigen::Matrix3Xd> &Corridor,
                                                            const Eigen::Vector2d &start,
                                                            const Eigen::Vector2d &goal,
                                                            const std::string connectivity = "8")
{
    GridPt start_idx, goal_idx, idx, start_border_idx, tmp_idx, revisit_idx;
    map_.getIndex(start, start_idx);
    map_.getIndex(goal, goal_idx);
    std::vector<GridPt> path; // TODO: Use freeman chain code to represent path
    idx = start_idx;
    if (!inCorridor(Corridor, map_, idx))
    {
        ROS_ERROR("Start point not in corridor!");
        return path;
    }
    // Find start border
    while (inCorridor(Corridor, map_, idx))
    {
        idx[0]++;
    }
    idx[0]--; // Back to last inCorridor
    start_border_idx = idx;
    path.push_back(start_border_idx);
    // Connectivity 8 Clockwise
    // 7 8 1
    // 6 * 2
    // 5 4 3
    std::vector<GridPt> c8_cw = {{1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}};

    // FIXME: Sometimes it stucks (loop)
    uint max_tries = 200;
    uint cnt = 0;
    do
    {
        cnt++;
        if (cnt > max_tries)
        {
            ROS_ERROR("getCorriderIntersectBorder() stucks!");
            printf("start(%d %d), now(%d %d)\n", start_border_idx[0], start_border_idx[1], idx[0], idx[1]);
            break;
        }

        bool out_corridor_flag = false;
        bool revisit_flag = false;
        for (int i = 0; i < 9; i++)
        {
            tmp_idx = idx + c8_cw.at(i);
            // Make use of short-circuit evaluation
            if (!out_corridor_flag && !inCorridor(Corridor, map_, tmp_idx))
            {
                out_corridor_flag = true;
            }
            if (out_corridor_flag && inCorridor(Corridor, map_, tmp_idx))
            {

                if (path.size() > 1 && tmp_idx[0] == path.at(path.size() - 2)[0] && tmp_idx[1] == path.at(path.size() - 2)[1])
                {
                    revisit_flag = true;
                    revisit_idx = tmp_idx;
                }
                else
                {
                    revisit_flag = false;
                    path.push_back(tmp_idx);
                    idx = tmp_idx;
                    break;
                }
            }
        }
        if (revisit_flag)
        {
            ROS_WARN("Revisit(%d %d)", revisit_idx[0], revisit_idx[1]);
            path.push_back(revisit_idx);
            idx = revisit_idx;
        }
        // timer_.milliSleep(100);
        // printf("now(%d %d)\n", idx[0], idx[1]);
        // drawSphereIdx(idx, 0.01);
    } while (idx[0] != start_border_idx[0] || idx[1] != start_border_idx[1]);

    return path;
}

/**
 * @brief Get the min length path using rope straining method
 * BUG: not properly implemented
 * @param[in] Border
 * @param[in] start
 * @param[in] goal
 * @param[out] path
 * @return true
 * @return false
 */
bool CVX_TrajOpt::minlengthPath(const std::vector<GridPt> &Border,
                                const Eigen::Vector2d &start,
                                const Eigen::Vector2d &goal,
                                std::vector<GridPt> &path)
{
    GridPt start_idx, goal_idx; // FIXME: is this appropriate?
    map_.getIndex(start, start_idx);
    map_.getIndex(goal, goal_idx);
    path.clear();
    path.push_back(start_idx);
    path.push_back(goal_idx);

    // TODO: use Border!
    std::vector<GridPt> concave_points;
    findConcavePoint(Border, concave_points);
    // std::vector<GridPt> ptsSideA, ptsSideB;
    // findConcavePoint(Border, start, goal, ptsSideA, ptsSideB);
    uint i = 0, ip1; // Concave point index
    int intersect_id = -1;
    uint last_update_cnt = 0;
    uint max_seg = 20;
    while (true)
    {
        ip1 = (i + 1) % concave_points.size();
        intersect_id = pathIntersect(path, concave_points.at(i), concave_points.at(ip1));
        if (intersect_id >= 0)
        {
            printf("i: %d, ip1: %d, intersect_id: %d\n", i, ip1, intersect_id);
            printf("p1: (%d %d), p2: (%d %d)\n", concave_points.at(i)[0], concave_points.at(i)[1], concave_points.at(ip1)[0], concave_points.at(ip1)[1]);
            printf("path_i: (%d %d), path_ip1: (%d %d)\n", path.at(intersect_id)[0], path.at(intersect_id)[1], path.at(intersect_id + 1)[0], path.at(intersect_id + 1)[1]);
            last_update_cnt = 0;
            path.insert(path.begin() + intersect_id + 1, concave_points.at(ip1));
        }

        i = ip1;
        last_update_cnt++;
        if (last_update_cnt > concave_points.size())
        {
            break;
        }
        if (path.size() > max_seg)
        {
            break;
        }
    }
    return true;
}

////////////////////
// Test

void CVX_TrajOpt::test_map()
{
    for (uint i = 0; i < map_.getLayers().size(); i++)
    {
        printf("%s\n", map_.getLayers()[i].c_str());
    }

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
        printf("%f\n", map_.at("elevation", *iterator));
    }
    return;
}

void CVX_TrajOpt::draw_vpoly_2DinHullPointset()
{
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
    visualizer_.visualizePolytope(CorridorBuf);

    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator)
    {
        if (inCorridor(CorridorBuf, map_, *iterator))
        {
            Eigen::Vector3d pos;
            Eigen::Vector2d posxy;
            pos[2] = map_.at("elevation", *iterator);
            map_.getPosition(*iterator, posxy);
            pos[0] = posxy.x();
            pos[1] = posxy.y();
            visualizer_.visualizeSphere(pos, 0.01);
        }
    }

    return;
}

void CVX_TrajOpt::drawCorriderIntersectBorder(const std::vector<Eigen::Matrix3Xd> &Corridor,
                                              const Eigen::Vector2d &start, const Eigen::Vector2d &goal)
{

    // Visualize start & goal
    GridPt start_idx, goal_idx;
    map_.getIndex(start, start_idx);
    drawSphereIdx(start_idx, 0.02);
    map_.getIndex(goal, goal_idx);
    drawSphereIdx(goal_idx, 0.02);

    // Get path
    std::vector<GridPt> path = getCorriderIntersectBorder(Corridor, start, goal);

    // Draw path
    std::vector<Eigen::Vector3d> path_pos;
    for (uint i = 0; i < path.size(); i++)
    {
        // drawSphereIdx(path.at(i));
        Eigen::Vector3d pos;
        Eigen::Vector2d posxy;
        pos[2] = map_.at("elevation", path.at(i));
        map_.getPosition(path.at(i), posxy);
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        path_pos.push_back(pos);
    }
    visualizer_.visualizeCurve(path_pos, MarkerStyle(1, 0, 0, 1, 0.01));
}

void CVX_TrajOpt::drawCorriderIntersectBorderTest()
{
    // clean
    visualizer_.deleteCurve();
    visualizer_.deleteSphere();

    Eigen::MatrixX3d waypoints(3, 3);
    waypoints << -0.5, 0.0, 0.2,
        0.0, 0.0, 0.4,
        0.5, 0.0, 0.2;
    // Eigen::Vector2d start(-0.4, -0.3), goal(0.4, -0.3);
    Eigen::Vector2d goal(0.4, -0.3);

    std::vector<Eigen::Matrix3Xd> RegionBuf;
    std::vector<Eigen::Matrix3Xd> CorridorBuf;

    for (int i = 0; i < waypoints.rows(); i++)
    {
        RegionBuf.push_back((vPoly.array().colwise() + (waypoints.transpose().col(i).array() + pos_shift.transpose().col(0).array())).eval());
        if (i > 0)
        {
            CorridorBuf.push_back(geo_utils::mergeVpoly(RegionBuf.at(i - 1), RegionBuf.at(i)));
        }
    }
    // visualizer.visualizePolytope(RegionBuf);
    visualizer_.visualizePolytope(CorridorBuf);

    drawCorriderIntersectBorder(CorridorBuf, start, goal);

    GridPolyLine Border = getCorriderIntersectBorder(CorridorBuf, start, goal);
    if (Border.size() < 3)
    {
        ROS_ERROR("Border.size() < 3");
        return;
    }
    GridPoints concave_pts;
    findConcavePoint(Border, concave_pts);
    for (uint i = 0; i < concave_pts.size(); i++)
    {
        drawSphereIdx(concave_pts.at(i), 0.02);
    }

    // Visiblity Graph
    GridPt start_grid, goal_grid; // FIXME: is this appropriate?
    map_.getIndex(start, start_grid);
    map_.getIndex(goal, goal_grid);
    VisibilityGraph vis_graph(Border, concave_pts, start_grid, goal_grid);
    uint size = vis_graph.size();
    for (uint i = 0; i < size; i++)
    {
        for (uint j = i + 1; j < size; j++)
        {
            if (vis_graph.isVisibile(i, j))
            {
                drawSegmentIdx(vis_graph.getPt(i), vis_graph.getPt(j));
            }
        }
    }

    std::vector<GridPt> path;
    bool ret = GCS_AStarSearch(vis_graph, path);
    // minlengthPath(Border, start, goal, path);
    // Draw path
    std::vector<Eigen::Vector3d> path_pos;
    for (uint i = 0; i < path.size(); i++)
    {
        // printf("path.at(%d): (%d %d)\n", i, path.at(i)[0], path.at(i)[1]);
        Eigen::Vector3d pos;
        Eigen::Vector2d posxy;
        pos[2] = map_.at("elevation", path.at(i));
        map_.getPosition(path.at(i), posxy);
        pos[0] = posxy.x();
        pos[1] = posxy.y();
        path_pos.push_back(pos);
    }
    visualizer_.visualizeCurve(path_pos, MarkerStyle(0, 1, 0, 1, 0.01));
}

void CVX_TrajOpt::segmentIntersectTest()
{
    GridPt p1, p2, q1, q2;
    p1 << 0, 0;
    p2 << 1, 1;
    q1 << 0, 1;
    q2 << 1, 0;
    segmentIntersect(p1, p2, q1, q2, true);
    return;
}