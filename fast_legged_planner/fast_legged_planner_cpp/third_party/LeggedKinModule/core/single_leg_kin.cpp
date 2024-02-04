/* related header files */
#include "single_leg_kin.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */
// Pinocchio
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/rnea.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/parsers/urdf.hpp"
/* internal project header files */

SingleLegKin::SingleLegKin(std::string urdf_file_path)
{
    pinocchio::urdf::buildModel(urdf_file_path, model_);
    data_ = pinocchio::Data(model_);
    joint_limits_ << -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI;
};

/**
 * @brief Check if the joints are within the joint limits
 *
 * @param joints Already redirected joint angles
 * @return true
 * @return false
 */
bool SingleLegKin::checkJointLimits(const Eigen::Vector3d &joints)
{
    for (size_t i = 0; i < 3; i++)
    {
        if (joints[i] < joint_limits_[2 * i] || joints[i] > joint_limits_[2 * i + 1])
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Inverse kinematics using IKFast
 *
 * @param pos expected position of the end effector in the base frame
 * @param sols solutions of the inverse kinematics with directed joint angles
 * @param approx whether to use dichotomy to find the solution (IKFast_warpper built-in)
 * @return true
 * @return false
 */
bool SingleLegKin::inverseKin(const Eigen::Vector3d &pos, std::vector<Eigen::Vector3d> &sols,
                              bool approx)
{
    Eigen::Vector3d target =
        mirror_offset_mat_ * rot_offset_.transpose() * (pos - pos_offset_) + origin_calib_;
    std::vector<double> target_vec;
    if (approx) // Use dichotomy provided by IKFast_wrapper
    {
        Eigen::Vector3d approx_vec =
            mirror_offset_mat_ * rot_offset_.transpose() * (ik_approx_point_ - pos_offset_) +
            origin_calib_;
        target_vec = {target[0], target[1], target[2], approx_vec[0], approx_vec[1], approx_vec[2]};
    }
    else
    {
        target_vec = {target[0], target[1], target[2]};
    }

    std::vector<std::vector<double>> ret = IKFast_trans3D(target_vec, approx);
    if (ret.size() == 0)
        return false;
    sols.resize(ret.size());
    for (size_t i = 0; i < ret.size(); i++)
    {
        sols[i] = Eigen::Map<Eigen::Vector3d>(ret[i].data());
        sols[i] = joint_dir_mat_ * sols[i]; // Redirect joint angles
    }
    return true;
}

/**
 * @brief Filter out the solutions that are out of joint limits & redirect joint angles
 *
 * @param sols
 */
void SingleLegKin::constraintFiltering(std::vector<Eigen::Vector3d> &sols)
{
    for (size_t i = 0; i < sols.size(); i++)
    {
        sols[i] = joint_dir_mat_ * sols[i];
        if (!checkJointLimits(sols[i]))
        {
            sols.erase(sols.begin() + i);
            i--;
        }
    }
}

// TODO: test
bool SingleLegKin::inverseKinConstraint(const Eigen::Vector3d &pos,
                                        std::vector<Eigen::Vector3d> &sols, bool approx, uint iter)
{
    Eigen::Vector3d target_pt =
        mirror_offset_mat_ * rot_offset_.transpose() * (pos - pos_offset_) + origin_calib_;
    Eigen::Vector3d approx_pt =
        mirror_offset_mat_ * rot_offset_.transpose() * (ik_approx_point_ - pos_offset_) +
        origin_calib_;
    Eigen::Vector3d tmp_pt;
    sols = IKFast_trans3D(target_pt);
    constraintFiltering(sols);
    if (sols.size() > 0)
        return true;
    else if (approx)
    {
        for (int i = 0; i < iter; ++i)
        {
            tmp_pt = (target_pt + approx_pt) / 2;
            sols = IKFast_trans3D(tmp_pt);
            constraintFiltering(sols);
            if (sols.size() > 0)
                approx_pt = tmp_pt;
            else
                target_pt = tmp_pt;
        }
        sols = IKFast_trans3D(approx_pt);
        constraintFiltering(sols);
        return sols.size() > 0;
    }
    return false;
}

bool SingleLegKin::inverseKinConstraint(const Eigen::Vector3d &pos, Eigen::Vector3d &sol,
                                        bool approx, uint iter)
{
    std::vector<Eigen::Vector3d> sols;
    inverseKinConstraint(pos, sols, approx, iter);
    if (sols.size() == 0)
    {
        return false;
    }
    sol = sols[0];
    return true;
}

bool SingleLegKin::forwardKin(const Eigen::Vector3d &joints, Eigen::Vector3d &pos)
{
    pinocchio::forwardKinematics(model_, data_, joint_dir_mat_ * joints);
    pos = pinocchio::updateFramePlacement(model_, data_, model_.getFrameId(end_effector_name_))
              .translation();
    pos = rot_offset_ * mirror_offset_mat_ * (pos - origin_calib_) + pos_offset_;
    return true;
}

bool SingleLegKin::forwardKinConstraint(const Eigen::Vector3d &joints, Eigen::Vector3d &pos)
{
    if (!checkJointLimits(joint_dir_mat_ * joints))
    {
        return false;
    }
    return forwardKin(joints, pos);
}
