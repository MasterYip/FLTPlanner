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

/* internal project header files */
#include "elspider_air_kin.h"
#include "fast_legged_planner/robot_interface/BaseRobotInterface.h"

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
public:
    ElSpiderKin robot_kin;
    ElSpiderAirInterface(const std::string &urdf, const std::vector<std::string> &package_dirs = {})
        : BaseRobotInterface(urdf, package_dirs)
    {
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
};
