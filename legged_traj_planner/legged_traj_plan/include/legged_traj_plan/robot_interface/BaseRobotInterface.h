/**
 * @file BaseRobotInterface.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-03
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <filesystem>
#include <iostream>
/* external project header files */
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
// include pinocchio before boost
#include "legged_traj_plan/FootState.h"
#include "legged_traj_search/geo_utils/polyhedra.hpp"
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
/* internal project header files */

class BaseRobotInterface {
private:
  pinocchio::Model model_;
  pinocchio::Data data_;

protected:
  std::vector<Polyhedra> foot_polyhedra_; // Defined in BASE frame
public:
  BaseRobotInterface(const std::string &urdf,
                     const std::vector<std::string> &package_dirs = {}) {
    // Parse URDF
    if (urdf.find("<") == 0) {
      std::string tmp_urdf =
          "/tmp/temp_urdf.urdf"; // Change to appropriate temporary file path
      std::ofstream temp_urdf_file(tmp_urdf);
      temp_urdf_file << urdf;
      temp_urdf_file.close();
      pinocchio::urdf::buildModel(tmp_urdf, model_);
    }
    // else if (urdf.substr(urdf.length() - 5) == ".urdf" &&
    // std::filesystem::exists(urdf)) // FIXME: c++17
    else if (urdf.substr(urdf.length() - 5) == ".urdf") {
      pinocchio::urdf::buildModel(urdf, model_);
    } else {
      throw std::invalid_argument("URDF file or string are not specified.");
    }
  }

  void update_kinematics(const Eigen::VectorXd &q) {
    pinocchio::forwardKinematics(model_, data_, q);
  }

  pinocchio::FrameIndex get_frameid(const std::string &frame_name) {
    return model_.getFrameId(frame_name);
  }

  pinocchio::SE3 get_frame_placement(const Eigen::VectorXd &q,
                                     const std::string &frame_name,
                                     bool update_kinematics = true) {
    // pinocchio::forwardKinematics(model_, data_, joint_dir_mat_ * q);
    pinocchio::forwardKinematics(model_, data_, q);
    return pinocchio::updateFramePlacement(model_, data_,
                                           model_.getFrameId(frame_name));
  }

  Polyhedra getFootPolyhedra(int index) const {
    return foot_polyhedra_.at(index);
  }

  std::vector<Polyhedra> getFootPolyhedra() const { return foot_polyhedra_; }

  // Virtual Kinematics Interface - to be implemented by derived classes
  virtual std::vector<double>
  IKFast_foots(const std::vector<Eigen::Vector3d> &footendpos) {
    throw std::runtime_error("IKFast_foots not implemented in derived class");
  }

  virtual Eigen::Vector3d IKFast_foot(const Eigen::Vector3d &footendpos,
                                      int index) {
    throw std::runtime_error("IKFast_foot not implemented in derived class");
  }

  virtual bool IKFast_foot(const Eigen::Vector3d &footendpos,
                           Eigen::Vector3d &q_result, int index) {
    throw std::runtime_error("IKFast_foot not implemented in derived class");
  }

  // Alternative name for compatibility - delegates to IKFast_foot
  virtual Eigen::Vector3d IK_foot(const Eigen::Vector3d &footendpos,
                                  int index) {
    return IKFast_foot(footendpos, index);
  }

  // Overload for constraint checking with boolean return
  virtual bool IK_foot(const Eigen::Vector3d &footendpos,
                       Eigen::Vector3d &q_result, int index) {
    return IKFast_foot(footendpos, q_result, index);
  }

  virtual Eigen::Vector3d FK_foot(const Eigen::Vector3d &q, int index) {
    throw std::runtime_error("FK_foot not implemented in derived class");
  }

  virtual Eigen::Vector3d FK_CollBall(const Eigen::Vector3d &q, int legIdx,
                                      int jointIdx) {
    throw std::runtime_error("FK_CollBall not implemented in derived class");
  }

  virtual Eigen::Matrix3Xd getJacobian(const Eigen::Vector3d &q, int index) {
    throw std::runtime_error("getJacobian not implemented in derived class");
  }

  virtual Eigen::Matrix3Xd getJacobianTimeVariation(const Eigen::Vector3d &q,
                                                    const Eigen::Vector3d &vel,
                                                    int index) {
    throw std::runtime_error(
        "getJacobianTimeVariation not implemented in derived class");
  }

  virtual Eigen::Matrix3Xd getJacobian_CollBall(const Eigen::Vector3d &q,
                                                int legIdx, int jointIdx) {
    throw std::runtime_error(
        "getJacobian_CollBall not implemented in derived class");
  }

  virtual Eigen::Matrix3d
  getJacobianTimeVariation_CollBall(const Eigen::Vector3d &q,
                                    const Eigen::Vector3d &vel, int legIdx,
                                    int jointIdx) {
    throw std::runtime_error(
        "getJacobianTimeVariation_CollBall not implemented in derived class");
  }

  virtual Eigen::Vector3d getNominalFoothold(int index) {
    throw std::runtime_error(
        "getNominalFoothold not implemented in derived class");
  }

  // Virtual Feedback Interface - to be implemented by derived classes
  // NOTE: Foot state in BASE frame
  virtual const legged_traj_plan::FootState &getFootStateFdb() const {
    throw std::runtime_error(
        "getFootStateFdb not implemented in derived class");
  }

  virtual const sensor_msgs::JointState &getJointStateFdb() const {
    throw std::runtime_error(
        "getJointStateFdb not implemented in derived class");
  }

  virtual const pinocchio::SE3 &getBodyPoseFdb() const {
    throw std::runtime_error("getBodyPoseFdb not implemented in derived class");
  }

  virtual const pinocchio::Motion &getBodyVelFdb() const {
    throw std::runtime_error("getBodyVelFdb not implemented in derived class");
  }

  // Virtual Command Interface - to be implemented by derived classes
  virtual void setBodyPoseCmd(const pinocchio::SE3 &body_pose) {
    throw std::runtime_error("setBodyPoseCmd not implemented in derived class");
  }

  virtual void setBodyVelCmd(const pinocchio::Motion &body_vel) {
    throw std::runtime_error("setBodyVelCmd not implemented in derived class");
  }

  virtual void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos) {
    throw std::runtime_error("setFootCmd not implemented in derived class");
  }

  virtual void setFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                          const std::vector<Eigen::Vector3d> &footendvel,
                          const std::vector<Eigen::Vector3d> &footendeffort,
                          const std::vector<bool> &contact) {
    throw std::runtime_error(
        "setFootCmd with velocity/effort not implemented in derived class");
  }

  virtual void setJointCmd(const std::vector<double> &q) {
    throw std::runtime_error(
        "setJointCmd (vector<double>) not implemented in derived class");
  }

  virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q) {
    throw std::runtime_error(
        "setJointCmd (vector<Vector3d>) not implemented in derived class");
  }

  virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q,
                           const std::vector<bool> &contact) {
    throw std::runtime_error(
        "setJointCmd with contact not implemented in derived class");
  }

  virtual void setJointCmd(const std::vector<Eigen::Vector3d> &q,
                           const std::vector<Eigen::Vector3d> &v,
                           const std::vector<Eigen::Vector3d> &tau,
                           const std::vector<bool> &contact) {
    throw std::runtime_error(
        "setJointCmd with velocity/torque not implemented in derived class");
  }

  // Debug
  void print_joints() {
    for (size_t i = 0; i < model_.njoints; ++i) {
      std::cout << i << " " << model_.names[i] << std::endl;
    }
  }

  void print_frames() {
    for (size_t i = 0; i < model_.nframes; ++i) {
      std::cout << i << " " << model_.frames[i].name << std::endl;
    }
  }
};
