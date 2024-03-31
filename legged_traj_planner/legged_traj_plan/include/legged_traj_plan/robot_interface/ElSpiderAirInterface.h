/**
 * @file ElSpiderAirInterface.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
#include <string>
/* external project header files */
#include "legged_traj_search/geo_utils/polyhedra.hpp"
/* internal project header files */
#include "elspider_air_kin.h"
#include "legged_traj_plan/robot_interface/BaseRobotInterface.h"

// Define JOINT_STATE_NAME and FOOT_LINK_NAME constants
const std::vector<std::string> JOINT_STATE_NAME = {"RF_HAA", "RF_HFE", "RF_KFE",
                                                   "RM_HAA", "RM_HFE", "RM_KFE",
                                                   "RB_HAA", "RB_HFE", "RB_KFE",
                                                   "LF_HAA", "LF_HFE", "LF_KFE",
                                                   "LM_HAA", "LM_HFE", "LM_KFE",
                                                   "LB_HAA", "LB_HFE", "LB_KFE"};

const std::vector<std::string> FOOT_LINK_NAME = {"RF_FOOT", "RM_FOOT", "RB_FOOT",
                                                 "LF_FOOT", "LM_FOOT", "LB_FOOT"};

const std::vector<std::string> SHADOW_JOINT_STATE_NAME = {"shadowRF_HAA", "shadowRF_HFE", "shadowRF_KFE",
                                                          "shadowRM_HAA", "shadowRM_HFE", "shadowRM_KFE",
                                                          "shadowRB_HAA", "shadowRB_HFE", "shadowRB_KFE",
                                                          "shadowLF_HAA", "shadowLF_HFE", "shadowLF_KFE",
                                                          "shadowLM_HAA", "shadowLM_HFE", "shadowLM_KFE",
                                                          "shadowLB_HAA", "shadowLB_HFE", "shadowLB_KFE"};

const std::vector<std::string> SHADOW_FOOT_LINK_NAME = {"shadowRF_FOOT", "shadowRM_FOOT", "shadowRB_FOOT",
                                                        "shadowLF_FOOT", "shadowLM_FOOT", "shadowLB_FOOT"};

class ElSpiderAirInterface : public BaseRobotInterface
{
private:
    std::vector<Polyhedra> foot_polyhedra_; // Defined in BASE frame

public:
    ElSpiderKin robot_kin;

    ElSpiderAirInterface(const std::string &urdf, const std::vector<std::string> &package_dirs = {})
        : BaseRobotInterface(urdf, package_dirs)
    {
        Eigen::MatrixX3d vertices(10, 3);
        vertices << 0.2412, -0.154, -0.1303,
            -0.07939, -0.1551, -0.1464,
            -0.0809, -0.1567, -0.3889,
            0.2556, -0.1674, -0.3545,
            -0.3199, -0.3958, 0.006312,
            -0.2209, -0.2967, -0.3344,
            0.3721, -0.2772, 0.02371,
            0.3527, -0.2589, -0.2644,
            0.05979, -0.4186, -0.2857,
            0.06059, -0.472, 0.1195;
        Eigen::Matrix3Xd vertices_transpose = vertices.transpose();
        for (int i = 0; i < 6; i++)
        {
            // TOOD: Transform
            foot_polyhedra_.emplace_back(Polyhedra(vertices_transpose));
        }
    }
    std::vector<double> IKFast_foots(const std::vector<Eigen::Vector3d> &footendpos)
    {
        std::vector<double> q;
        for (int i = 0; i < 6; i++)
        {
            Eigen::Vector3d q_i;
            robot_kin.inverseKinConstraint(footendpos[i], q_i, i);
            q.push_back(q_i[0]);
            q.push_back(q_i[1]);
            q.push_back(q_i[2]);
        }
        return q;
    }

    ElSpiderKin &getRobotKin()
    {
        return robot_kin;
    }

    Polyhedra getFootPolyhedra(int index) const
    {
        return foot_polyhedra_[index];
    }
};
