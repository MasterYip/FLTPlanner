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
// BUG: Smaller convex hull will cause unreachable output
inline Polyhedra genLegPolyRegion(int index)
{
    Eigen::Matrix3Xd hull;
    switch (index)
    {
    case 0:
        hull.resize(3, 10);
        hull << 0.5412, 0.36059, 0.3, -0.0199, 0.0791, 0.35979, 0.3, 0.6527, 0.6721, 0.5556,
            -0.234, -0.552, -0.29, -0.4758, -0.3767, -0.4986, -0.245, -0.3389, -0.3572, -0.2474,
            -0.1203, 0.1295, -0.1364, 0.016312, -0.3244, -0.2757, -0.3789, -0.2544, 0.03371, -0.3445;
        break;
    case 1:
        hull.resize(3, 10);
        hull << 0.06059, 0.3527, 0.3721, -0.3199, 6.22971e-16, 0.2412, 0.2556, -0.2209, 0.05979, 3.56833e-15,
            -0.612, -0.3989, -0.4172, -0.5358, -0.35, -0.294, -0.3074, -0.4367, -0.5586, -0.305,
            0.1295, -0.2544, 0.03371, 0.016312, -0.1364, -0.1203, -0.3445, -0.3244, -0.2757, -0.3789;
        break;
    case 2:
        hull.resize(3, 10);
        hull << -0.5412, -0.36059, -0.3, 0.0199, -0.0791, -0.35979, -0.3, -0.6527, -0.6721, -0.5556,
            -0.234, -0.552, -0.29, -0.4758, -0.3767, -0.4986, -0.245, -0.3389, -0.3572, -0.2474,
            -0.1203, 0.1295, -0.1364, 0.016312, -0.3244, -0.2757, -0.3789, -0.2544, 0.03371, -0.3445;
        break;
    case 3:
        hull.resize(3, 10);
        hull << 0.5412, 0.36059, 0.3, -0.0199, 0.0791, 0.35979, 0.3, 0.6527, 0.6721, 0.5556,
            0.234, 0.552, 0.29, 0.4758, 0.3767, 0.4986, 0.245, 0.3389, 0.3572, 0.2474,
            -0.1203, 0.1295, -0.1364, 0.016312, -0.3244, -0.2757, -0.3789, -0.2544, 0.03371, -0.3445;
        break;
    case 4:
        hull.resize(3, 10);
        hull << 0.06059, 0.3527, 0.3721, -0.3199, -8.4136e-15, 0.2412, 0.2556, -0.2209, 0.05979, -8.40464e-14,
            0.612, 0.3989, 0.4172, 0.5358, 0.35, 0.294, 0.3074, 0.4367, 0.5586, 0.305,
            0.1295, -0.2544, 0.03371, 0.016312, -0.1364, -0.1203, -0.3445, -0.3244, -0.2757, -0.3789;
        break;
    case 5:
        hull.resize(3, 10);
        hull << -0.5412, -0.36059, -0.3, 0.0199, -0.0791, -0.35979, -0.3, -0.6527, -0.6721, -0.5556,
            0.234, 0.552, 0.29, 0.4758, 0.3767, 0.4986, 0.245, 0.3389, 0.3572, 0.2474,
            -0.1203, 0.1295, -0.1364, 0.016312, -0.3244, -0.2757, -0.3789, -0.2544, 0.03371, -0.3445;
        break;
    default:
        throw std::invalid_argument("Invalid leg index");
    }
    return Polyhedra(hull);
}

class ElSpiderAirInterface : public BaseRobotInterface
{

public:
    ElSpiderKin robot_kin;

    ElSpiderAirInterface(const std::string &urdf, const std::vector<std::string> &package_dirs = {})
        : BaseRobotInterface(urdf, package_dirs)
    {
        // Foot convex hull
        Eigen::Matrix3Xd FootHull(3, 10);
        FootHull << 0.2412, -0.07939, -0.0809, 0.2556, -0.3199, -0.2209, 0.3721, 0.3527, 0.05979, 0.06059,
            -0.154, -0.1551, -0.1567, -0.1674, -0.3958, -0.2967, -0.2772, -0.2589, -0.4186, -0.472,
            -0.1303, -0.1464, -0.3889, -0.3545, 0.006312, -0.3344, 0.02371, -0.2644, -0.2857, 0.1195;
        Eigen::Matrix3Xd pos_shift(3, 6);
        // FIXME: Installation shift
        // pos_shift << 0.3, 0.0, -0.3, 0.3, 0.0, -0.3,
        //     0.06, 0.0, 0.06, -0.06, 0.0, -0.06,
        //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        pos_shift << 0.3, 0.0, -0.3, 0.3, 0.0, -0.3,
            -0.04, -0.1, -0.04, 0.04, 0.1, 0.04,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::Matrix3Xd mirror(3, 6);
        mirror << 1, 1, -1, 1, 1, -1,
            1, 1, 1, -1, -1, -1,
            1, 1, 1, 1, 1, 1;
        for (int i = 0; i < 6; i++)
        {
            // TOOD: Transform
            // Eigen::Matrix3Xd hull = FootHull;
            // hull.row(0) *= mirror(0, i);
            // hull.row(1) *= mirror(1, i);
            // hull.row(2) *= mirror(2, i);
            // foot_polyhedra_.emplace_back(Polyhedra((hull.colwise() + pos_shift.col(i)).eval()));
            // BUG: This one may cuase unreachable output
            foot_polyhedra_.emplace_back(genLegPolyRegion(i));
        }
    }

    std::vector<double> IKFast_foots(const std::vector<Eigen::Vector3d> &footendpos)
    {
        std::vector<double> q;
        for (int i = 0; i < 6; i++)
        {
            Eigen::Vector3d q_i;
            robot_kin.inverseKinConstraint(footendpos[i], q_i, i);
            // robot_kin.inverseKin(footendpos[i], q_i, i);
            q.push_back(q_i[0]);
            q.push_back(q_i[1]);
            q.push_back(q_i[2]);
        }
        return q;
    }

    Eigen::Vector3d IKFast_foot(const Eigen::Vector3d &footendpos, int index)
    {
        Eigen::Vector3d q_i;
        robot_kin.inverseKinConstraint(footendpos, q_i, index);
        // robot_kin.inverseKin(footendpos, q_i, index);
        return q_i;
    }

    Eigen::Vector3d FK_foot(const Eigen::Vector3d &q, int index)
    {
        Eigen::Vector3d footendpos;
        robot_kin.forwardKinConstraint(q, footendpos, index);
        return footendpos;
    }

    Eigen::Vector3d FK_CollBall(const Eigen::Vector3d &q, int legIdx, int jointIdx)
    {
        Eigen::Vector3d pos;
        robot_kin.forwardKin(q, pos, legIdx, jointIdx);
        return pos;
    }

    Eigen::Matrix3Xd getJacobian(const Eigen::Vector3d &q, int index)
    {
        Eigen::Matrix3Xd J(3, 3);
        robot_kin.getJacobian(q, J, index);
        return J;
    }

    Eigen::Matrix3Xd getJacobianTimeVariation(const Eigen::Vector3d &q, const Eigen::Vector3d &vel, int index)
    {
        Eigen::Matrix3Xd J_dot(3, 3);
        robot_kin.getJacobianTimeVariation(q, vel, J_dot, index);
        return J_dot;
    }

    Eigen::Matrix3Xd getJacobian_CollBall(const Eigen::Vector3d &q, int legIdx, int jointIdx)
    {
        Eigen::Matrix3Xd J(3, 3);
        robot_kin.getJacobian(q, J, legIdx, jointIdx);
        return J;
    }

    Eigen::Matrix3d getJacobianTimeVariation_CollBall(const Eigen::Vector3d &q, const Eigen::Vector3d &vel,
                                                      int legIdx, int jointIdx)
    {
        Eigen::Matrix3Xd J_dot(3, 3);
        robot_kin.getJacobianTimeVariation(q, vel, J_dot, legIdx, jointIdx);
        return J_dot;
    }

    ElSpiderKin &getRobotKin()
    {
        return robot_kin;
    }
};
