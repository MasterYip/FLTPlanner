/**
 * @file visibility.hpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-28
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

#include "legged_traj_search/geo_utils/geo_utils_2d.hpp"

using namespace geo_utils_2d;

/**
 * @brief Check if two points are visible to each other
 * TODO:
 * 1. what if p1 p2 can be outside the border
 * 2. Make it can go through overlapping border
 * IMPORTANT: Border should be clockwise, Border.at(0) != Border.at(-1)
 * @param Border
 * @param p1 Least one on or Inside the Border
 * @param p2 Least one on or Inside the Border
 * @return true
 * @return false
 */
inline bool visiblityCheck(const GridPolyLine &Border, const GridPt &p1, const GridPt &p2)
{
    int p1_idx = -1, p2_idx = -1; // check if p1 and p2 are border point
    if (p1.isApprox(p2))
        return true;
    // Judge Intersection
    for (uint i = 0; i < Border.size(); i++)
    {
        if (Border.at(i).isApprox(p1))
            p1_idx = i;
        if (Border.at(i).isApprox(p2))
            p2_idx = i;
        IntersectType intersect_type = segmentIntersect(Border.at(i), Border.at((i + 1) % Border.size()), p1, p2);
        if (intersect_type == IntersectType::Middle || intersect_type == IntersectType::EndMid || intersect_type == IntersectType::MidEnd)
            return false;
    }

    // Judge if p1 and p2 are visible to each other from outside (should not be counted as visible)
    if (p1_idx != -1)
    {
        int plus1 = (p1_idx + 1) % Border.size();
        int minus1 = (p1_idx - 1 + Border.size()) % Border.size();
        GridPt vec1 = Border.at(p1_idx) - Border.at(minus1);
        GridPt vec2 = Border.at(plus1) - Border.at(p1_idx);
        GridPt vec_p1p2 = p2 - p1;
        if ((crossProd(vec1, vec2) > 0 && (crossProd(vec1, vec_p1p2) > 0 && crossProd(vec2, vec_p1p2) > 0)) || // Concave Point
            (crossProd(vec1, vec2) < 0 && (crossProd(vec1, vec_p1p2) > 0 || crossProd(vec2, vec_p1p2) > 0)) || // Convex Point
            (crossProd(vec1, vec2) == 0 && crossProd(vec1, vec_p1p2) > 0))                                     // Straight Line
        {
            return false;
        }
    }
    if (p2_idx != -1)
    {
        int plus1 = (p2_idx + 1) % Border.size();
        int minus1 = (p2_idx - 1 + Border.size()) % Border.size();
        GridPt vec1 = Border.at(p2_idx) - Border.at(minus1);
        GridPt vec2 = Border.at(plus1) - Border.at(p2_idx);
        GridPt vec_p2p1 = p1 - p2;
        if ((crossProd(vec1, vec2) > 0 && (crossProd(vec1, vec_p2p1) > 0 && crossProd(vec2, vec_p2p1) > 0)) || // Concave Point
            (crossProd(vec1, vec2) < 0 && (crossProd(vec1, vec_p2p1) > 0 || crossProd(vec2, vec_p2p1) > 0)) || // Convex Point
            (crossProd(vec1, vec2) == 0 && crossProd(vec1, vec_p2p1) > 0))                                     // Straight Line
        {
            return false;
        }
    }
    return true;
}

inline bool visiblityCheck(const GridPolyLine &Border, const Point &p1, const Point &p2)
{
    int p1_idx = -1, p2_idx = -1; // check if p1 and p2 are border point
    if (p1.isApprox(p2))
        return true;
    // Judge Intersection
    for (uint i = 0; i < Border.size(); i++)
    {
        if (p1.isApprox(Border.at(i).cast<double>()))
            p1_idx = i;
        if (p2.isApprox(Border.at(i).cast<double>()))
            p2_idx = i;
        IntersectType intersect_type = segmentIntersect(Border.at(i).cast<double>(), Border.at((i + 1) % Border.size()).cast<double>(), p1, p2);
        if (intersect_type == IntersectType::Middle || intersect_type == IntersectType::EndMid || intersect_type == IntersectType::MidEnd)
            return false;
    }

    // Judge if p1 and p2 are visible to each other from outside (should not be counted as visible)
    if (p1_idx != -1)
    {
        int plus1 = (p1_idx + 1) % Border.size();
        int minus1 = (p1_idx - 1 + Border.size()) % Border.size();
        Point vec1 = (Border.at(p1_idx) - Border.at(minus1)).cast<double>();
        Point vec2 = (Border.at(plus1) - Border.at(p1_idx)).cast<double>();
        Point vec_p1p2 = p2 - p1;
        if ((crossProd(vec1, vec2) > 0 && (crossProd(vec1, vec_p1p2) > 0 && crossProd(vec2, vec_p1p2) > 0)) || // Concave Point
            (crossProd(vec1, vec2) < 0 && (crossProd(vec1, vec_p1p2) > 0 || crossProd(vec2, vec_p1p2) > 0)) || // Convex Point
            (crossProd(vec1, vec2) == 0 && crossProd(vec1, vec_p1p2) > 0))                                     // Straight Line
        {
            return false;
        }
    }
    if (p2_idx != -1)
    {
        int plus1 = (p2_idx + 1) % Border.size();
        int minus1 = (p2_idx - 1 + Border.size()) % Border.size();
        Point vec1 = (Border.at(p2_idx) - Border.at(minus1)).cast<double>();
        Point vec2 = (Border.at(plus1) - Border.at(p2_idx)).cast<double>();
        Point vec_p2p1 = p1 - p2;
        if ((crossProd(vec1, vec2) > 0 && (crossProd(vec1, vec_p2p1) > 0 && crossProd(vec2, vec_p2p1) > 0)) || // Concave Point
            (crossProd(vec1, vec2) < 0 && (crossProd(vec1, vec_p2p1) > 0 || crossProd(vec2, vec_p2p1) > 0)) || // Convex Point
            (crossProd(vec1, vec2) == 0 && crossProd(vec1, vec_p2p1) > 0))                                     // Straight Line
        {
            return false;
        }
    }
    return true;
}

class VisibilityGraph
{
private:
    GridPolyLine Border_;
    GridPoints concavePts_;
    GridPt start_;
    GridPt goal_;
    Point startf_;
    Point goalf_;
    // Order: start, goal, concavePts
    // 0: unknown, 1: visible, -1: invisible
    Eigen::MatrixXi visibilityGraph_;
    uint size_;
    // Start Goal use float
    bool useFloat_ = false;

public:
    VisibilityGraph(void) = default;

    VisibilityGraph(const GridPolyLine &Border, const GridPoints &concavePts, const GridPt &start, const GridPt &goal)
        : Border_(Border), concavePts_(concavePts), start_(start), goal_(goal)
    {
        visibilityGraph_.resize(concavePts_.size() + 2, concavePts_.size() + 2);
        visibilityGraph_.setZero();
        size_ = concavePts_.size() + 2;
    }

    VisibilityGraph(const GridPolyLine &Border, const GridPoints &concavePts, const Point &startf, const Point &goalf)
        : Border_(Border), concavePts_(concavePts), startf_(startf), goalf_(goalf)
    {
        visibilityGraph_.resize(concavePts_.size() + 2, concavePts_.size() + 2);
        visibilityGraph_.setZero();
        size_ = concavePts_.size() + 2;
        // round to int
        start_ << std::lround(startf[0]), std::lround(startf[1]);
        goal_ << std::lround(goalf[0]), std::lround(goalf[1]);
        useFloat_ = true;
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
     * @param i index of [start, goal, concavePts]
     * @param j index of [start, goal, concavePts]
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
            if (useFloat_)
            {
                if (i == 0)
                {
                    if (j == 1)
                    {
                        if (visiblityCheck(Border_, startf_, goalf_))
                            visibilityGraph_(i, j) = 1;
                        else
                            visibilityGraph_(i, j) = -1;
                    }
                    else
                    {
                        if (visiblityCheck(Border_, startf_, concavePts_.at(j - 2).cast<double>()))
                            visibilityGraph_(i, j) = 1;
                        else
                            visibilityGraph_(i, j) = -1;
                    }
                }
                else if (i == 1)
                {
                    if (visiblityCheck(Border_, goalf_, concavePts_.at(j - 2).cast<double>()))
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
            else
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
        }
        return visibilityGraph_(i, j) == 1;
    }
};
