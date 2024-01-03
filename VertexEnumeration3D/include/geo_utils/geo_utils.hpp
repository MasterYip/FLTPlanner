/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include "quickhull.hpp"
#include "sdlp.hpp"

#include <Eigen/Eigen>

#include <cfloat>
#include <cstdint>
#include <set>
#include <chrono>

namespace geo_utils
{

    /**
     * @brief Find the interior point of a convex polyhedron
     * @note
     * - Using sdlp to find the interior point.
     * - Each row of hPoly is defined by `h0, h1, h2, h3` as `h0*x + h1*y + h2*z + h3 <= 0`
     * - LP problem is defined as `max w, [H0, H1, H2, 1][x,y,z,w]T <= [-H3]`, where w can be deem as the "shift" of half-space.
     * This formulation can find the point that a hpoly finally collapse to.
     * @param[in] hPoly
     * @param[out] interior
     * @return true
     * @return false
     */
    inline bool findInterior(const Eigen::MatrixX4d &hPoly,
                             Eigen::Vector3d &interior)
    {
        const int m = hPoly.rows();

        Eigen::MatrixX4d A(m, 4);
        Eigen::VectorXd b(m);
        Eigen::Vector4d c, x;
        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        A.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm; // Normalize
        A.rightCols<1>().setConstant(1.0);
        b = -hPoly.rightCols<1>().array() / hNorm;
        c.setZero();
        c(3) = -1.0; // What is the purpose of this?

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
        interior = x.head<3>();

        return minmaxsd < 0.0 && !std::isinf(minmaxsd);
    }

    /**
     * @brief Check if two convex polyhedra overlap
     * @note
     * - Compute the intersection of two polyhedra and check if the intersection is empty.
     * @param hPoly0
     * @param hPoly1
     * @param eps epsilon for numerical stability
     * @return true
     * @return false
     */
    inline bool overlap(const Eigen::MatrixX4d &hPoly0,
                        const Eigen::MatrixX4d &hPoly1,
                        const double eps = 1.0e-6)

    {
        const int m = hPoly0.rows();
        const int n = hPoly1.rows();
        // Compute the intersection of two polyhedra
        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);
        A.leftCols<3>().topRows(m) = hPoly0.leftCols<3>();
        A.leftCols<3>().bottomRows(n) = hPoly1.leftCols<3>();
        A.rightCols<1>().setConstant(1.0);
        b.topRows(m) = -hPoly0.rightCols<1>();
        b.bottomRows(n) = -hPoly1.rightCols<1>();
        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }

    /**
     * @brief Check if a point is inside a convex polyhedron
     *
     * @param hPoly
     * @param point
     * @param eps
     * @return true
     * @return false
     */
    inline bool inHpoly(const Eigen::MatrixX4d &hPoly,
                        const Eigen::Vector3d &point,
                        const double eps = 0)
    {
        return (hPoly.leftCols<3>() * point + hPoly.rightCols<1>()).maxCoeff() <= eps;
    }

    /**
     * @brief Convert a vPoly to hPoly
     * @note
     * Each row of hPoly is defined by `h0, h1, h2, h3` as `h0*x + h1*y + h2*z + h3 <= 0`
     * @param vPoly
     * @return const Eigen::MatrixX4d
     */
    inline const Eigen::MatrixX4d vpoly2hpoly(const Eigen::Matrix3Xd &vPoly)
    {
        Eigen::Matrix3Xd mesh;
        // Get the convex hull(that includes the points) in mesh
        quickhull::QuickHull<double> qh;
        const auto cvxHull = qh.getConvexHull(vPoly.data(),
                                              vPoly.cols(),
                                              false, false);
        const auto &idBuffer = cvxHull.getIndexBuffer();
        const auto &vtBuffer = cvxHull.getVertexBuffer();
        int ids = idBuffer.size();
        // The mesh is represented by triangles(3 vertices) in right hand rule USING idBuffer
        mesh.resize(3, ids);
        quickhull::Vector3<double> v;
        for (int i = 0; i < ids; i++)
        {
            v = vtBuffer[idBuffer[i]];
            mesh(0, i) = v.x;
            mesh(1, i) = v.y;
            mesh(2, i) = v.z;
        }

        // Obtain the half space intersection(H-rep) form from the mesh
        Eigen::MatrixX4d hPoly(ids / 3, 4);
        Eigen::Vector3d normal, point, edge0, edge1;
        for (int i = 0; i < ids / 3; i++)
        {
            point = mesh.col(3 * i + 1);
            edge0 = point - mesh.col(3 * i);
            edge1 = mesh.col(3 * i + 2) - point;
            normal = edge0.cross(edge1).normalized();
            hPoly(i, 0) = normal(0);
            hPoly(i, 1) = normal(1);
            hPoly(i, 2) = normal(2);
            hPoly(i, 3) = -normal.dot(point);
        }
        return hPoly;
    }

    /**
     * @brief Merge two vPoly into one (combine vertices & quickhull)
     *
     * @param vPoly1
     * @param vPoly2
     * @return const Eigen::Matrix3Xd
     */
    inline const Eigen::Matrix3Xd mergeVpoly(const Eigen::Matrix3Xd &vPoly1, const Eigen::Matrix3Xd &vPoly2)
    {
        Eigen::Matrix3Xd vertices;
        Eigen::Matrix3Xd vtcombined(3, vPoly1.cols() + vPoly2.cols());
        vtcombined.leftCols(vPoly1.cols()) = vPoly1;
        vtcombined.rightCols(vPoly2.cols()) = vPoly2;
        // Get the convex hull(that includes the points) in mesh
        quickhull::QuickHull<double> qh;
        const auto cvxHull = qh.getConvexHull(vtcombined.data(),
                                              vtcombined.cols(),
                                              false, false);
        const auto &vtBuffer = cvxHull.getVertexBuffer();
        int vts = vtBuffer.size();
        vertices.resize(3, vts);
        quickhull::Vector3<double> v;
        for (int i = 0; i < vts; i++)
        {
            v = vtBuffer[i];
            vertices(0, i) = v.x;
            vertices(1, i) = v.y;
            vertices(2, i) = v.z;
        }
        return vertices;
    }

    /**
     * @brief
     */
    struct filterLess
    {
        inline bool operator()(const Eigen::Vector3d &l,
                               const Eigen::Vector3d &r)
        {
            return l(0) < r(0) ||
                   (l(0) == r(0) &&
                    (l(1) < r(1) ||
                     (l(1) == r(1) &&
                      l(2) < r(2))));
        }
    };

    /**
     * @brief
     *
     * @param[in] rV
     * @param[in] epsilon
     * @param[out] fV
     */
    inline void filterVs(const Eigen::Matrix3Xd &rV,
                         const double &epsilon,
                         Eigen::Matrix3Xd &fV)
    {
        const double mag = std::max(fabs(rV.maxCoeff()), fabs(rV.minCoeff()));
        const double res = mag * std::max(fabs(epsilon) / mag, DBL_EPSILON);
        std::set<Eigen::Vector3d, filterLess> filter;
        fV = rV;
        int offset = 0;
        Eigen::Vector3d quanti;
        for (int i = 0; i < rV.cols(); i++)
        {
            quanti = (rV.col(i) / res).array().round();
            if (filter.find(quanti) == filter.end())
            {
                filter.insert(quanti);
                fV.col(offset) = rV.col(i);
                offset++;
            }
        }
        fV = fV.leftCols(offset).eval();
        return;
    }

    /**
     * @brief Enumerate the vertices of a convex polyhedron
     * @note TODO: Add notes & explaination
     * Each row of hPoly is defined by `h0, h1, h2, h3` as `h0*x + h1*y + h2*z + h3 <= 0`
     * @param[in] hPoly
     * @param[in] inner Interior point of hPoly
     * @param[out] vPoly
     * @param[in] epsilon {1.0e-6}
     */
    inline void enumerateVs(const Eigen::MatrixX4d &hPoly,
                            const Eigen::Vector3d &inner,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        const Eigen::VectorXd b = -hPoly.rightCols<1>() - hPoly.leftCols<3>() * inner;
        const Eigen::Matrix<double, 3, -1, Eigen::ColMajor> A =
            (hPoly.leftCols<3>().array().colwise() / b.array()).transpose();

        quickhull::QuickHull<double> qh;
        const double qhullEps = std::min(epsilon, quickhull::defaultEps<double>());
        // CCW(counter clock-wise?) is false because the normal in quickhull towards interior
        const auto cvxHull = qh.getConvexHull(A.data(), A.cols(), false, true, qhullEps);
        const auto &idBuffer = cvxHull.getIndexBuffer();
        const int hNum = idBuffer.size() / 3;
        Eigen::Matrix3Xd rV(3, hNum);
        Eigen::Vector3d normal, point, edge0, edge1;
        for (int i = 0; i < hNum; i++)
        {
            point = A.col(idBuffer[3 * i + 1]);
            edge0 = point - A.col(idBuffer[3 * i]);
            edge1 = A.col(idBuffer[3 * i + 2]) - point;
            normal = edge0.cross(edge1); // cross in CW gives an outter normal
            rV.col(i) = normal / normal.dot(point);
        }
        filterVs(rV, epsilon, vPoly);
        vPoly = (vPoly.array().colwise() + inner.array()).eval(); // translate back
        return;
    }

    /**
     * @brief enumerateVs overload
     *
     * @param[in] hPoly
     * @param[out] vPoly
     * @param[in] epsilon {1.0e-6}
     * @return true
     * @return false
     */
    inline bool enumerateVs(const Eigen::MatrixX4d &hPoly,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        Eigen::Vector3d inner;
        if (findInterior(hPoly, inner))
        {
            enumerateVs(hPoly, inner, vPoly, epsilon);
            return true;
        }
        else
        {
            return false;
        }
    }

    [[deprecated("Use intersectVpoly instead")]] inline bool intersectVpoly_Legacy(const Eigen::Matrix3Xd &vPoly1,
                                                                                   const Eigen::Matrix3Xd &vPoly2,
                                                                                   Eigen::Matrix3Xd &vPolyIntersect, double eps = 1e-6)
    {
        Eigen::MatrixX4d hPoly1 = vpoly2hpoly(vPoly1);
        Eigen::MatrixX4d hPoly2 = vpoly2hpoly(vPoly2);
        Eigen::Matrix3Xd vPolyCombined(3, vPoly1.cols() + vPoly2.cols());
        Eigen::Matrix3Xd vPolyTmp(3, vPoly1.cols() + vPoly2.cols());
        int cnt = 0;
        vPolyCombined.leftCols(vPoly1.cols()) = vPoly1;
        vPolyCombined.rightCols(vPoly2.cols()) = vPoly2;
        for (int i = 0; i < vPolyCombined.cols(); i++)
        {
            if (geo_utils::inHpoly(hPoly1, vPolyCombined.col(i), eps) &&
                geo_utils::inHpoly(hPoly2, vPolyCombined.col(i), eps))
            {
                vPolyTmp.col(cnt) = vPolyCombined.col(i);
                cnt++;
            }
        }
        vPolyIntersect = vPolyTmp.leftCols(cnt);
        if (cnt < 4)
            return false;
        else
            return true;
    }

    inline bool intersectHpoly(const Eigen::MatrixX4d &hPoly1,
                               const Eigen::MatrixX4d &hPoly2,
                               Eigen::Matrix3Xd &vPolyIntersect)
    {
        Eigen::MatrixX4d hPolyCombined(hPoly1.rows() + hPoly2.rows(), 4);
        hPolyCombined.topRows(hPoly1.rows()) = hPoly1;
        hPolyCombined.bottomRows(hPoly2.rows()) = hPoly2;
        return enumerateVs(hPolyCombined, vPolyIntersect);
    }

    /**
     * @brief Intersection of two vPoly
     * @param[in] vPoly1
     * @param[in] vPoly2
     * @param[out] vPolyIntersect
     * @param[in] eps
     * @return true
     * @return false
     */
    inline bool intersectVpoly(const Eigen::Matrix3Xd &vPoly1,
                               const Eigen::Matrix3Xd &vPoly2,
                               Eigen::Matrix3Xd &vPolyIntersect)
    {
        Eigen::MatrixX4d hPoly1 = vpoly2hpoly(vPoly1);
        Eigen::MatrixX4d hPoly2 = vpoly2hpoly(vPoly2);
        return intersectHpoly(hPoly1, hPoly2, vPolyIntersect);
    }
} // namespace geo_utils

#endif
