// -*- coding: utf-8 -*-
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
/* external project header files */
#if 1
#include <Eigen/Core>
#endif
/* internal project header files */

namespace ikfast_leg
{
    bool IKFast_trans3D(const double trans[3], double solret[10][3]);

    std::vector<std::vector<double>> IKFast_trans3D(const std::vector<double> trans,
                                                    bool approx = false);

#if 1
    std::vector<Eigen::Vector3d> IKFast_trans3D(const Eigen::Vector3d trans);
#endif

} // namespace ikfast_leg
