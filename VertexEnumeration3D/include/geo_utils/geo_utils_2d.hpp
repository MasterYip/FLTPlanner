#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
/* external project header files */
#include <Eigen/Eigen>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
/* internal project header files */

using Index = Eigen::Array2i;
using PolyLine = std::vector<Eigen::Vector2d>;
using GridPolyLine = std::vector<Index>;
using GridPoints = std::vector<Index>;

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;
typedef K::Line_2 Line_2;
typedef K::Intersect_2 Intersect_2;

namespace geo_utils_2d
{
    // Statement (Temp)
    bool findConcavePoint(const GridPolyLine &Border,
                          const Index &start,
                          const Index &goal,
                          GridPoints &ptsSideA,
                          GridPoints &ptsSideB);
    bool findConcavePoint(const GridPolyLine &Border, GridPoints &concavePts, bool isClockwise);

    // Utils

    int crossProd(const Index &a, const Index &b)
    {
        return a[0] * b[1] - a[1] * b[0];
    }

    int innerProd(const Index &a, const Index &b)
    {
        return a[0] * b[0] + a[1] * b[1];
    }

    uint manhattanLength(const Index &start, const Index &goal)
    {
        // TODO: prevent using std::abs
        return std::abs(start[0] - goal[0]) + std::abs(start[1] - goal[1]);
    }

    // Concave Points

    bool isConcavePoint(const Index &pt, const Index &pt_prev, const Index &pt_next, const bool clockwise = true)
    {
        Index tmp_dir1, tmp_dir2;
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
        Index tmp_dir1, tmp_dir2;
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
                          const Index &start,
                          const Index &goal,
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
        Index tmp_dir1, tmp_dir2;
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

    // Intersection

    /**
     * @brief segment intersect detection (CGAL)
     *
     * @param p1
     * @param p2
     * @param q1
     * @param q2
     * @return true intersect
     * @return false not intersect OR overlap
     */
    bool segmentIntersect(const Index &p1, const Index &p2,
                          const Index &q1, const Index &q2, const bool verbose = false)
    {
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
                return false; // Overlap
            else
                return true;
        }
        return false;
    }

    /**
     * @brief Path intersect detection
     * FIXME: bugs
     * @param path
     * @param p1
     * @param p2
     * @return uint
     * if segment path(i,i+1) intersect with segment (p1,p2), return i;
     * else return -1
     */
    uint pathIntersect(const GridPolyLine &path, const Index &p1, const Index &p2)
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

    /**
     * @brief [[deprecated]] Check if a point is on the left side of a path
     * @note This problem is not well defined
     * @param path
     * @param pt
     * @return true: LHS or on the path
     */
    [[deprecated]] bool checkPointSideLHS(const GridPolyLine &path, const Index &pt)
    {
        Index ui, p;
        Index tmp;
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

    [[deprecated]] bool checkPointSideRHS(const GridPolyLine &path, const Index &pt)
    {
        Index ui, p;
        Index tmp;
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