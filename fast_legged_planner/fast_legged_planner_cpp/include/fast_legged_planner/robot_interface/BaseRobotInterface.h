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
#include <iostream>
#include <filesystem>
/* external project header files */
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

/* internal project header files */

class BaseRobotInterface
{
private:
    pinocchio::Model model_;
    pinocchio::Data data_;

public:
    BaseRobotInterface(const std::string &urdf, const std::vector<std::string> &package_dirs = {})
    {
        // Parse URDF
        if (urdf.find("<") == 0)
        {
            std::string tmp_urdf = "/tmp/temp_urdf.urdf"; // Change to appropriate temporary file path
            std::ofstream temp_urdf_file(tmp_urdf);
            temp_urdf_file << urdf;
            temp_urdf_file.close();
            pinocchio::urdf::buildModel(tmp_urdf, model_);
        }
        // else if (urdf.substr(urdf.length() - 5) == ".urdf" && std::filesystem::exists(urdf)) // FIXME: c++17
        else if (urdf.substr(urdf.length() - 5) == ".urdf")
        {
            pinocchio::urdf::buildModel(urdf, model_);
        }
        else
        {
            throw std::invalid_argument("URDF file or string are not specified.");
        }
    }

    void update_kinematics(const Eigen::VectorXd &q)
    {
        pinocchio::forwardKinematics(model_, data_, q);
    }

    pinocchio::FrameIndex get_frameid(const std::string &frame_name)
    {
        return model_.getFrameId(frame_name);
    }

    pinocchio::SE3 get_frame_placement(const Eigen::VectorXd &q, const std::string &frame_name, bool update_kinematics = true)
    {
        // pinocchio::forwardKinematics(model_, data_, joint_dir_mat_ * q);
        pinocchio::forwardKinematics(model_, data_, q);
        return pinocchio::updateFramePlacement(model_, data_, model_.getFrameId(frame_name));
    }

    // Debug
    void print_joints()
    {
        for (size_t i = 0; i < model_.njoints; ++i)
        {
            std::cout << i << " " << model_.names[i] << std::endl;
        }
    }

    void print_frames()
    {
        for (size_t i = 0; i < model_.nframes; ++i)
        {
            std::cout << i << " " << model_.frames[i].name << std::endl;
        }
    }
};
