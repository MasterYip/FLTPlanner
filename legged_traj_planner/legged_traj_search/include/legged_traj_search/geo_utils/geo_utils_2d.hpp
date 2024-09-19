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
// #define USE_CGAL
#ifdef USE_CGAL
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/number_utils.h>
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

    enum class IntersectType
    {
        None,   // Not intersect
        Middle, // Intersect in the middle of segment p & q
        End,    // Intersect at the end of segment p & q
        MidEnd, // Intersect in the middle of segment p, and at the end of segment q
        EndMid, // Intersect at the end of segment p, and in the middle of segment q
        Overlap // Overlap
    };
#ifdef USE_CGAL
    // Intersection
    /**
     * @brief segment intersect detection (CGAL)
     * TODO: test needed
     * TODO: Optimization needed
     * @param p1
     * @param p2
     * @param q1
     * @param q2
     * @return IntersectType
     */
    inline IntersectType segmentIntersect(const Point &p1, const Point &p2,
                                 const Point &q1, const Point &q2, const bool verbose = false)
    {
        // Judge if intersect at the end or overlap
        uint tmp = 1;
        if (p1.isApprox(q1) || p1.isApprox(q2))
            tmp++;
        if (p2.isApprox(q1) || p2.isApprox(q2))
            tmp++;
        if (tmp == 2)
            return IntersectType::End;
        else if (tmp == 3)
            return IntersectType::Overlap;

        Segment_2 s1(Point_2(p1[0], p1[1]), Point_2(p2[0], p2[1]));
        Segment_2 s2(Point_2(q1[0], q1[1]), Point_2(q2[0], q2[1]));
        const auto result = intersection(s1, s2);
        // Verbose
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
                return IntersectType::Overlap;
            else
            {
                const Point_2 *p = boost::get<Point_2>(&*result);
                Point intersectPt(CGAL::to_double(p->x()), CGAL::to_double(p->y()));
                if (intersectPt.isApprox(p1) || intersectPt.isApprox(p2))
                    return IntersectType::EndMid;
                else if (intersectPt.isApprox(q1) || intersectPt.isApprox(q2))
                    return IntersectType::MidEnd;
                else
                    return IntersectType::Middle;
            }
        }
        return IntersectType::None;
    }
#else

    inline bool approx(double a, double b, double eps = 1e-6)
    {
        return std::abs(a - b) < eps;
    }

    /**
     * @brief Construct a new in Section object
     * 
     * @param p1 
     * @param p2 
     * @param pt
     * @return 1-in, 0-on, -1-out
     */
    inline int inSection(const Point &p1, const Point &p2, const Point &pt)
    {
        Point ub(std::max(p1[0], p2[0]), std::max(p1[1], p2[1]));
        Point lb(std::min(p1[0], p2[0]), std::min(p1[1], p2[1]));
        if (pt[0] < ub[0] && pt[0] > lb[0] || pt[1] < ub[1] && pt[1] > lb[1])
            return 1;
        else if ((approx(pt[0], ub[0]) || approx(pt[0], lb[0])) && (approx(pt[1], ub[1]) || approx(pt[1], lb[1])))
            return 0;
        else
            return -1;
    }

    // Intersection
    /**
     * @brief segment intersect detection (CGAL)
     * TODO: Optimization needed
     * @param p1
     * @param p2
     * @param q1
     * @param q2
     * @return IntersectType
     */
    inline IntersectType segmentIntersect(const Point &p1, const Point &p2,
                                 const Point &q1, const Point &q2, const bool verbose = false)
    {
        // Judge if intersect at the end or overlap
        uint tmp = 1;
        if (p1.isApprox(q1) || p1.isApprox(q2))
            tmp++;
        if (p2.isApprox(q1) || p2.isApprox(q2))
            tmp++;
        if (tmp == 2)
            return IntersectType::End;
        else if (tmp == 3)
            return IntersectType::Overlap;

        // Calculate line equation in the form of Ax + By = C, A = y2 - y1, B = x1 - x2, C = A * x1 + B * y1
        Eigen::Vector3d coef1(p2[1] - p1[1], p1[0] - p2[0], 0);
        Eigen::Vector3d coef2(q2[1] - q1[1], q1[0] - q2[0], 0);
        coef1[2] = coef1[0] * p1[0] + coef1[1] * p1[1];
        coef2[2] = coef2[0] * q1[0] + coef2[1] * q1[1];

        // parallel check
        if (coef1[0] * coef2[1] == coef2[0] * coef1[1])
            return IntersectType::None;

        // find intersection point
        Eigen::Vector2d pt;
        pt[0] = (coef2[1] * coef1[2] - coef1[1] * coef2[2]) / (coef1[0] * coef2[1] - coef2[0] * coef1[1]);
        pt[1] = (coef1[0] * coef2[2] - coef2[0] * coef1[2]) / (coef1[0] * coef2[1] - coef2[0] * coef1[1]);
        int insec1 = inSection(p1, p2, pt);
        int insec2 = inSection(q1, q2, pt);
        if (insec1 == 1 && insec2 == 1)
            return IntersectType::Middle;
        else if (insec1 == 0 && insec2 == 1)
            return IntersectType::EndMid;
        else if (insec1 == 1 && insec2 == 0)
            return IntersectType::MidEnd;
        else
            return IntersectType::None;
    }

#endif

    inline IntersectType segmentIntersect(const GridPt &p1, const GridPt &p2,
                                 const GridPt &q1, const GridPt &q2, const bool verbose = false)
    {
        return segmentIntersect(Point(p1[0], p1[1]), Point(p2[0], p2[1]),
                                Point(q1[0], q1[1]), Point(q2[0], q2[1]), verbose);
    }


} // namespace geo_utils_2d