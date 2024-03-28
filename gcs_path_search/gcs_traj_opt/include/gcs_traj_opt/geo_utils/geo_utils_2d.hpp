/**
 * @file geo_utils_2d.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-02-23
 *
 * @copyright Copyright (c) 2024
 *
 */

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

namespace geo_utils_2d
{
    using Point = Eigen::Array2d;
    using GridPt = Eigen::Array2i;
    using PolyLine = std::vector<Point>;
    using GridPolyLine = std::vector<GridPt>;
    using GridPoints = std::vector<GridPt>;

    inline bool findConcavePoint(const GridPolyLine &Border,
                                 const GridPt &start,
                                 const GridPt &goal,
                                 GridPoints &ptsSideA,
                                 GridPoints &ptsSideB);
    inline bool findConcavePoint(const GridPolyLine &Border, GridPoints &concavePts, bool isClockwise);

    // Utils

    inline int crossProd(const GridPt &a, const GridPt &b)
    {
        return a[0] * b[1] - a[1] * b[0];
    }

    inline int innerProd(const GridPt &a, const GridPt &b)
    {
        return a[0] * b[0] + a[1] * b[1];
    }

    inline uint manhattanLength(const GridPt &start, const GridPt &goal)
    {
        // TODO: prevent using std::abs
        return std::abs(start[0] - goal[0]) + std::abs(start[1] - goal[1]);
    }

    // Concave Points

    inline bool isConcavePoint(const GridPt &pt, const GridPt &pt_prev, const GridPt &pt_next, const bool clockwise = true)
    {
        GridPt tmp_dir1, tmp_dir2;
        tmp_dir1 = pt - pt_prev;
        tmp_dir2 = pt_next - pt;
        int dir_cross_prod = tmp_dir1[0] * tmp_dir2[1] - tmp_dir1[1] * tmp_dir2[0];
        return ((dir_cross_prod > 0 && clockwise) || (dir_cross_prod < 0 && !clockwise));
    }

    // NOTE: start point check is wrong if border[end] = border[0]
    inline bool findConcavePoint(const GridPolyLine &Border, GridPoints &concavePts, bool isClockwise = true)
    {
        concavePts.clear();
        if (Border.size() < 2)
        {
            std::cerr << "Border.size() < 2" << std::endl;
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
     * @brief segment intersect detection (CGAL)
     * TODO: test needed
     * TODO: Optimization needed
     * FIXME: How to deal with Point&GridPt Mix?
     * @param p1
     * @param p2
     * @param q1
     * @param q2
     * @return 0 not intersect
     * @return 1 intersect in the middle (crossing / endpoint touch the other segment)
     * @return 2 intersect at the end (at least 1 of endpoints are the same)
     * @return 3 overlap
     */
    inline uint segmentIntersect(const Point &p1, const Point &p2,
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
            // FIXME: test needed
            if (boost::get<Segment_2>(&*result))
                return 3; // Overlap
            else
                return 1;
        }
        return 0;
    }

    inline uint segmentIntersect(const GridPt &p1, const GridPt &p2,
                                 const GridPt &q1, const GridPt &q2, const bool verbose = false)
    {
        return segmentIntersect(Point(p1[0], p1[1]), Point(p2[0], p2[1]),
                                Point(q1[0], q1[1]), Point(q2[0], q2[1]), verbose);
    }

#endif

} // namespace geo_utils_2d