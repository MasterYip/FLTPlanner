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

inline Polyhedra genLegPolyRegion(int index)
{
    Eigen::Matrix3Xd hull;
    switch (index)
    {
    case 0:
        hull.resize(3, 17);
        hull << 0.5412, 0.36059, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.35979, 0.6527, 0.6721, 0.6721, 0.6721, 0.5556,
            -0.234, -0.552, -0.238001, -0.236357, -0.234965, -0.245753, -0.427829, -0.531855, -0.492608, -0.455263, -0.312837, -0.4986, -0.3389, -0.3572, -0.3572, -0.3572, -0.2474,
            -0.1203, 0.1295, -0.374719, -0.346063, -0.134422, -0.121559, 0.0233491, 0.0995765, -0.198954, -0.293014, -0.348899, -0.2757, -0.2544, 0.03371, 0.03371, 0.03371, -0.3445;

        break;
    case 1:
        hull.resize(3, 20);
        hull << 0.06059, 0.22061, 0.22061, 0.22061, -0.1, -0.1, -0.1, -0.07939, 0.22061, 0.22061, 0.22061, 0.22061, -0.1, -0.1, 0.22061, 0.22061, -0.1, -0.1, 0.05979, -0.0809,
            -0.612, -0.495262, -0.511933, -0.470918, -0.498112, -0.315726, -0.315723, -0.2951, -0.294071, -0.294173, -0.376231, -0.443163, -0.579839, -0.549005, -0.306287, -0.352288, -0.489205, -0.3158, -0.5586, -0.2967,
            0.1295, -0.0808032, 0.0802935, -0.264005, 0.0218406, -0.123314, -0.163781, -0.1364, -0.121334, -0.136831, -0.0233676, 0.0299013, 0.0817278, -0.152809, -0.348077, -0.332206, -0.303424, -0.371465, -0.2757, -0.3789;
        break;
    case 2:
        hull.resize(3, 17);
        hull << -0.5412, -0.36059, -0.22061, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15, -0.35979, -0.2191, -0.6527, -0.6721, -0.6721, -0.6721, -0.5556,
            -0.234, -0.552, -0.2351, -0.305755, -0.305766, -0.446681, -0.509826, -0.486002, -0.407491, -0.3058, -0.4986, -0.2367, -0.3389, -0.3572, -0.3572, -0.3572, -0.2474,
            -0.1203, 0.1295, -0.1364, -0.230207, -0.0915661, 0.0205836, 0.0668538, -0.114355, -0.312099, -0.352, -0.2757, -0.3789, -0.2544, 0.03371, 0.03371, 0.03371, -0.3445;
        break;
    case 3:
        hull.resize(3, 17);
        hull << 0.5412, 0.36059, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.26, 0.35979, 0.6527, 0.6721, 0.6721, 0.6721, 0.5556,
            0.234, 0.552, 0.238001, 0.236357, 0.234965, 0.245753, 0.427829, 0.531855, 0.492608, 0.455263, 0.312837, 0.4986, 0.3389, 0.3572, 0.3572, 0.3572, 0.2474,
            -0.1203, 0.1295, -0.374719, -0.346063, -0.134422, -0.121559, 0.0233491, 0.0995765, -0.198954, -0.293014, -0.348899, -0.2757, -0.2544, 0.03371, 0.03371, 0.03371, -0.3445;
        break;
    case 4:
        hull.resize(3, 20);
        hull << 0.06059, 0.22061, 0.22061, 0.22061, -0.1, -0.1, -0.1, -0.07939, 0.22061, 0.22061, 0.22061, 0.22061, -0.1, -0.1, 0.22061, 0.22061, -0.1, -0.1, 0.05979, -0.0809,
            0.612, 0.495262, 0.511933, 0.470918, 0.498112, 0.315726, 0.315723, 0.2951, 0.294071, 0.294173, 0.376231, 0.443163, 0.579839, 0.549005, 0.306287, 0.352288, 0.489205, 0.3158, 0.5586, 0.2967,
            0.1295, -0.0808032, 0.0802935, -0.264005, 0.0218406, -0.123314, -0.163781, -0.1364, -0.121334, -0.136831, -0.0233676, 0.0299013, 0.0817278, -0.152809, -0.348077, -0.332206, -0.303424, -0.371465, -0.2757, -0.3789;
        break;
    case 5:
        hull.resize(3, 17);
        hull << -0.5412, -0.36059, -0.22061, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15, -0.35979, -0.2191, -0.6527, -0.6721, -0.6721, -0.6721, -0.5556,
            0.234, 0.552, 0.2351, 0.305755, 0.305766, 0.446681, 0.509826, 0.486002, 0.407491, 0.3058, 0.4986, 0.2367, 0.3389, 0.3572, 0.3572, 0.3572, 0.2474,
            -0.1203, 0.1295, -0.1364, -0.230207, -0.0915661, 0.0205836, 0.0668538, -0.114355, -0.312099, -0.352, -0.2757, -0.3789, -0.2544, 0.03371, 0.03371, 0.03371, -0.3445;
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
        // // Foot convex hull
        // Eigen::Matrix3Xd FootHull(3, 10);
        // FootHull << 0.2412, -0.07939, -0.0809, 0.2556, -0.3199, -0.2209, 0.3721, 0.3527, 0.05979, 0.06059,
        //     -0.154, -0.1551, -0.1567, -0.1674, -0.3958, -0.2967, -0.2772, -0.2589, -0.4186, -0.472,
        //     -0.1303, -0.1464, -0.3889, -0.3545, 0.006312, -0.3344, 0.02371, -0.2644, -0.2857, 0.1195;
        // Eigen::Matrix3Xd pos_shift(3, 6);
        // // FIXME: Installation shift
        // // pos_shift << 0.3, 0.0, -0.3, 0.3, 0.0, -0.3,
        // //     0.06, 0.0, 0.06, -0.06, 0.0, -0.06,
        // //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // pos_shift << 0.3, 0.0, -0.3, 0.3, 0.0, -0.3,
        //     -0.04, -0.1, -0.04, 0.04, 0.1, 0.04,
        //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // Eigen::Matrix3Xd mirror(3, 6);
        // mirror << 1, 1, -1, 1, 1, -1,
        //     1, 1, 1, -1, -1, -1,
        //     1, 1, 1, 1, 1, 1;
        for (int i = 0; i < 6; i++)
        {
            // TOOD: Transform
            // Eigen::Matrix3Xd hull = FootHull;
            // hull.row(0) *= mirror(0, i);
            // hull.row(1) *= mirror(1, i);
            // hull.row(2) *= mirror(2, i);
            // foot_polyhedra_.emplace_back(Polyhedra((hull.colwise() + pos_shift.col(i)).eval()));
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

    Eigen::Matrix3Xd getJacobian(const Eigen::Vector3d &q, int index)
    {
        Eigen::Matrix3Xd J(3, 3);
        robot_kin.getJacobian(q, J, index);
        return J;
    }

    ElSpiderKin &getRobotKin()
    {
        return robot_kin;
    }
};
