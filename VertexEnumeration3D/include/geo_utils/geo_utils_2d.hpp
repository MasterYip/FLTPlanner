
#pragma once
/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
/* external project header files */
#include <Eigen/Eigen>
#define USE_CGAL
#ifdef USE_CGAL
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;
typedef K::Line_2 Line_2;
typedef K::Intersect_2 Intersect_2;
#endif
/* internal project header files */
using Point = Eigen::Array2f;
using GridPt = Eigen::Array2i;
using PolyLine = std::vector<Eigen::Array2f>;
using GridPolyLine = std::vector<GridPt>;
using GridPoints = std::vector<GridPt>;

namespace geo_utils_2d
{
    // Statement (Temp)
    bool findConcavePoint(const GridPolyLine &Border,
                          const GridPt &start,
                          const GridPt &goal,
                          GridPoints &ptsSideA,
                          GridPoints &ptsSideB);
    bool findConcavePoint(const GridPolyLine &Border, GridPoints &concavePts, bool isClockwise);

    // Utils

    int crossProd(const GridPt &a, const GridPt &b)
    {
        return a[0] * b[1] - a[1] * b[0];
    }

    int innerProd(const GridPt &a, const GridPt &b)
    {
        return a[0] * b[0] + a[1] * b[1];
    }

    uint manhattanLength(const GridPt &start, const GridPt &goal)
    {
        // TODO: prevent using std::abs
        return std::abs(start[0] - goal[0]) + std::abs(start[1] - goal[1]);
    }

    // Concave Points

    bool isConcavePoint(const GridPt &pt, const GridPt &pt_prev, const GridPt &pt_next, const bool clockwise = true)
    {
        GridPt tmp_dir1, tmp_dir2;
        tmp_dir1 = pt - pt_prev;
        tmp_dir2 = pt_next - pt;
        int dir_cross_prod = tmp_dir1[0] * tmp_dir2[1] - tmp_dir1[1] * tmp_dir2[0];
        return ((dir_cross_prod > 0 && clockwise) || (dir_cross_prod < 0 && !clockwise));
    }

    bool findConcavePoint(const GridPolyLine &Border, GridPoints &concavePts, bool isClockwise = true)
    {
        concavePts.clear();
        if (Border.size() < 2)
        {
            ROS_ERROR("Border.size() < 2");
            return false;
        }
        // Find concave(concave towards the interior) point
        GridPt tmp_dir1, tmp_dir2;
        for (uint i = 0; i < Border.size(); i++)
        {
            uint im1 = (i - 1 + Border.size()) % Border.size();
            uint ip1 = (i + 1) % Border.size();
            if (isConcavePoint(Border.at(i), Border.at(im1), Border.at(ip1), isClockwise))
            {
                concavePts.push_back(Border.at(i));
            }
        }
        return true;
    }

    /**
     * @brief
     *
     * @param[in] Border
     * @param[in] start
     * @param[in] goal
     * @param[out] ptsSideA
     * @param[out] ptsSideB
     * @return true
     * @return false
     */
    bool findConcavePoint(const GridPolyLine &Border,
                          const GridPt &start,
                          const GridPt &goal,
                          GridPoints &ptsSideA,
                          GridPoints &ptsSideB)
    {
        ptsSideA.clear();
        ptsSideB.clear();
        if (Border.size() < 2)
        {
            ROS_ERROR("Border.size() < 2");
            return false;
        }
        // Step1: Find segment point
        // FIXME: seg_point should not be concave point? Maybe not necessary
        uint seg_point_start = 0, seg_point_goal = 0;
        uint mindis_start = manhattanLength(Border.at(0), start);
        uint mindis_goal = manhattanLength(Border.at(0), goal);
        uint dis_start, dis_goal;
        for (uint i = 1; i < Border.size(); i++)
        {
            dis_start = manhattanLength(Border.at(i), start);
            dis_goal = manhattanLength(Border.at(i), goal);
            if (dis_start < mindis_start)
            {
                mindis_start = dis_start;
                seg_point_start = i;
            }
            if (dis_goal < mindis_goal)
            {
                mindis_goal = dis_goal;
                seg_point_goal = i;
            }
        }
        // Draw segment point
        // drawSphereIdx(Border.at(seg_point_start), 0.02);
        // drawSphereIdx(Border.at(seg_point_goal), 0.02);

        // Step2: Find concave(concave towards the interior) point
        GridPt tmp_dir1, tmp_dir2;
        // |0--(B)--S----(A)---G---(B)--| tail
        bool in_sideA = !(seg_point_start < seg_point_goal && seg_point_start > 0);
        int dir_cross_prod;
        for (uint i = 0; i < Border.size(); i++) // NOTE: Border is clockwise
        {
            uint im1 = (i - 1 + Border.size()) % Border.size();
            uint ip1 = (i + 1) % Border.size();
            tmp_dir1 = Border.at(i) - Border.at(im1); // FIXME:
            tmp_dir2 = Border.at(ip1) - Border.at(i);
            dir_cross_prod = tmp_dir1[0] * tmp_dir2[1] - tmp_dir1[1] * tmp_dir2[0];
            if (i == seg_point_start)
                in_sideA = true;
            if (i == seg_point_goal)
                in_sideA = false;
            if (dir_cross_prod > 0)
            {
                if (in_sideA)
                    ptsSideA.push_back(Border.at(i));
                else
                    ptsSideB.push_back(Border.at(i));
            }
        }
        return true;
    }

#ifdef USE_CGAL

    enum class IntersectType
    {
        None,
        Middle,
        End,
        Overlap
    };

    // Intersection
    /**
     * @brief segment intersect detection (CGAL)[disabled to save complie time]
     * TODO: test needed
     * TODO: Optimization needed
     * FIXME: How to deal with Point&GridPt Mix?
     * @param p1
     * @param p2
     * @param q1
     * @param q2
     * @return 0 not intersect
     * @return 1 intersect in the middle
     * @return 2 intersect at the end
     * @return 3 overlap
     */
    uint segmentIntersect(const Point &p1, const Point &p2,
                          const Point &q1, const Point &q2, const bool verbose = false)
    {
        // Judge if intersect at the end or overlap
        uint tmp = 1;
        if (p1.isApprox(q1) || p1.isApprox(q2))
            tmp++;
        if (p2.isApprox(q1) || p2.isApprox(q2))
            tmp++;
        if (tmp > 1)
            return tmp;

        Segment_2 s1(Point_2(p1[0], p1[1]), Point_2(p2[0], p2[1]));
        Segment_2 s2(Point_2(q1[0], q1[1]), Point_2(q2[0], q2[1]));
        const auto result = intersection(s1, s2);
        if (result && verbose)
        {
            if (const Segment_2 *s = boost::get<Segment_2>(&*result))
            {
                std::cout << *s << std::endl;
            }
            else
            {
                const Point_2 *p = boost::get<Point_2>(&*result);
                std::cout << *p << std::endl;
            }
        }
        if (result)
        {
            if (const Segment_2 *s = boost::get<Segment_2>(&*result))
                return 3; // Overlap
            else
                return 1;
        }
        return 0;
    }

    uint segmentIntersect(const GridPt &p1, const GridPt &p2,
                          const GridPt &q1, const GridPt &q2, const bool verbose = false)
    {
        return segmentIntersect(Point(p1[0], p1[1]), Point(p2[0], p2[1]),
                                Point(q1[0], q1[1]), Point(q2[0], q2[1]), verbose);
    }

    // TODO: Test robustness
    /**
     * @brief Check if two points are visible to each other
     *
     * @param Border
     * @param p1 On or Inside the Border
     * @param p2 On or Inside the Border
     * @return true
     * @return false
     */
    bool visiblityCheck(const GridPolyLine &Border, const GridPt &p1, const GridPt &p2)
    {
        int p1_idx = -1, p2_idx = -1;
        for (uint i = 0; i < Border.size() - 1; i++)
        {
            if (Border.at(i).isApprox(p1))
                p1_idx = i;
            if (Border.at(i).isApprox(p2))
                p2_idx = i;
            if (segmentIntersect(Border.at(i), Border.at(i + 1), p1, p2) == 1)
            {
                return false;
            }
        }
        // Judge if p1 and p2 are visible to each other from outside
        if (p1_idx != -1)
        {
            int p1_idxp1 = (p1_idx + 1) % Border.size();
            GridPt vec_border = Border.at(p1_idxp1) - Border.at(p1_idx);
            GridPt vec_p1p2 = p2 - p1;
            if (crossProd(vec_border, vec_p1p2) > 0)
            {
                return false;
            }
        }
        if (p2_idx != -1)
        {
            int p2_idxp2 = (p2_idx + 1) % Border.size();
            GridPt vec_border = Border.at(p2_idxp2) - Border.at(p2_idx);
            GridPt vec_p2p1 = p1 - p2;
            if (crossProd(vec_border, vec_p2p1) > 0)
            {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Path intersect detection
     * BUG: bugs
     * @param path
     * @param p1
     * @param p2
     * @return int
     * if segment path(i,i+1) intersect with segment (p1,p2), return i;
     * else return -1
     */
    int pathIntersect(const GridPolyLine &path, const GridPt &p1, const GridPt &p2)
    {
        for (uint i = 0; i < path.size() - 1; i++)
        {
            if (segmentIntersect(path.at(i), path.at(i + 1), p1, p2))
            {
                return i;
            }
        }
        return -1;
    }

    class VisibilityGraph
    {
    private:
        GridPolyLine Border_;
        GridPoints concavePts_;
        GridPt start_;
        GridPt goal_;
        // Order: start, goal, concavePts
        // 0: unknown, 1: visible, -1: invisible
        Eigen::MatrixXi visibilityGraph_;
        uint size_;

    public:
        VisibilityGraph(const GridPolyLine &Border, const GridPoints &concavePts, const GridPt &start, const GridPt &goal)
            : Border_(Border), concavePts_(concavePts), start_(start), goal_(goal)
        {
            visibilityGraph_.resize(concavePts_.size() + 2, concavePts_.size() + 2);
            visibilityGraph_.setZero();
            size_ = concavePts_.size() + 2;
        }
        ~VisibilityGraph() {}

        uint size() const
        {
            return size_;
        }

        GridPt getPt(uint i) const
        {
            if (i >= size_)
            {
                printf("i out of range\n");
                return GridPt::Zero();
            }
            if (i == 0)
                return start_;
            else if (i == 1)
                return goal_;
            else
                return concavePts_.at(i - 2);
        }

        /**
         * @brief Update All visibility graph matrix
         *
         */
        void updateAll()
        {
            for (uint i = 0; i < size_; i++)
            {
                for (uint j = i + 1; j < size_; j++)
                {
                    isVisibile(i, j);
                }
            }
        }

        /**
         * @brief Check if two points are visible
         *
         * @param i index of [start, concavePts, goal]
         * @param j index of [start, concavePts, goal]
         * @return true
         * @return false
         */
        bool isVisibile(uint i, uint j)
        {
            if (i >= size_ || j >= size_)
            {
                printf("i or j out of range\n");
                return false;
            }
            if (i > j) // swap
            {
                uint tmp = i;
                i = j;
                j = tmp;
            }
            if (i == j)
            {
                return true;
            }

            if (visibilityGraph_(i, j) == 0)
            {
                if (i == 0)
                {
                    if (j == 1)
                    {
                        if (visiblityCheck(Border_, start_, goal_))
                            visibilityGraph_(i, j) = 1;
                        else
                            visibilityGraph_(i, j) = -1;
                    }
                    else
                    {
                        if (visiblityCheck(Border_, start_, concavePts_.at(j - 2)))
                            visibilityGraph_(i, j) = 1;
                        else
                            visibilityGraph_(i, j) = -1;
                    }
                }
                else if (i == 1)
                {
                    if (visiblityCheck(Border_, goal_, concavePts_.at(j - 2)))
                        visibilityGraph_(i, j) = 1;
                    else
                        visibilityGraph_(i, j) = -1;
                }
                else
                {
                    if (visiblityCheck(Border_, concavePts_.at(i - 2), concavePts_.at(j - 2)))
                        visibilityGraph_(i, j) = 1;
                    else
                        visibilityGraph_(i, j) = -1;
                }
            }
            return visibilityGraph_(i, j) == 1;
        }
    };

#endif // USE_CGAL

    /**
     * @brief [[deprecated]] Check if a point is on the left side of a path
     * @note This problem is not well defined
     * @param path
     * @param pt
     * @return true: LHS or on the path
     */
    [[deprecated]] bool checkPointSideLHS(const GridPolyLine &path, const GridPt &pt)
    {
        GridPt ui, p;
        GridPt tmp;
        double ai;
        for (uint i = 0; i < path.size() - 1; i++)
        {
            ui = path.at(i + 1) - path.at(i);
            p = pt - path.at(i);
            ai = 1.0 * innerProd(ui, p) / innerProd(ui, ui);
            if ((i == 0 && ai <= 1) || (i == path.size() - 2 && ai >= 0) || (ai >= 0 && ai <= 1 && i < path.size() - 2 && i > 0))
            {
                tmp << ui[0] * ai, ui[1] * ai;
                if (crossProd(ui, p - tmp) >= 0)
                {
                    return true;
                }
            }
        }
        return false;
    }

    [[deprecated]] bool checkPointSideRHS(const GridPolyLine &path, const GridPt &pt)
    {
        GridPt ui, p;
        GridPt tmp;
        double ai;
        for (uint i = 0; i < path.size() - 1; i++)
        {
            ui = path.at(i + 1) - path.at(i);
            p = pt - path.at(i);
            ai = 1.0 * innerProd(ui, p) / innerProd(ui, ui);
            if ((i == 0 && ai <= 1) || (i == path.size() - 2 && ai >= 0) || (ai >= 0 && ai <= 1 && i < path.size() - 2 && i > 0))
            {
                tmp << ui[0] * ai, ui[1] * ai;
                if (crossProd(ui, p - tmp) <= 0)
                {
                    return true;
                }
            }
        }
        return false;
    }

} // namespace geo_utils_2d