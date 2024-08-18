/**
 * @file CfgStompTask.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

/**
 * @file simple_optimization_task.h
 * @brief A simple task for showing how to use STOMP
 *
 * @author Jorge Nicho
 * @date Dec 14, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
#include "legged_traj_plan/utils/Spline.h"
#include "legged_traj_plan/swing_traj_planner/SwingTrajPlannerBase.h"
#include "legged_traj_plan/utils/Geometry.h"
#include "StompCollisionPenalty.h"
#include "StompLegLimitPenalty.h"
#include <stomp/task.h>

class StompTask : public stomp::Task
{
protected:
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    SwingTrajPlannerConfig config_;

    std::vector<double> std_dev_; /**< Standard deviation used for generating noisy parameters */
    Eigen::MatrixXd smoothing_M_; /**< Matrix used for smoothing the trajectory */

    StompCollisionPenalty collision_penalty_;
    Eigen::Vector3d p0_, p1_; // Cartesian position

    // Visualizer
    ros::NodeHandle nh_;
    ros::Rate rate_ = ros::Rate(5);
    std::shared_ptr<GCSVisualizer> visualizer_;
    bool enable_vis_ = false;

public:
    StompTask(SwingTrajPlannerConfig config,
              std::shared_ptr<GridMapInterface> gridmap_interface,
              std::shared_ptr<GCSVisualizer> visualizer = nullptr)
        : config_(config), gridmap_interface_(gridmap_interface),
          collision_penalty_(gridmap_interface, visualizer),
          visualizer_(visualizer)
    {
        if (visualizer != nullptr)
            enable_vis_ = true;

        rate_ = ros::Rate(config.optVisRate);

        collision_penalty_.setupParams(config_);

        // generate smoothing matrix
        std_dev_ = {config_.stompStdDev1, config_.stompStdDev2, config_.stompStdDev3};
        stomp::generateSmoothingMatrix(config_.stompNumTimesteps, config_.trajTime / (config_.stompNumTimesteps - 1), smoothing_M_);
        srand(time(0));
    };

    void setup(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1)
    {
        p0_ = p0;
        p1_ = p1;
        collision_penalty_.setExcludeBall(p0, p1);
    };

    void setupVis(std::shared_ptr<GCSVisualizer> visualizer)
    {
        if (visualizer != nullptr)
        {
            enable_vis_ = true;
            visualizer_ = visualizer;
            // collision_penalty_.setupVis(visualizer);
        }
    }

    //! [CfgStompTask Inherit]
    /**
     * @brief Generates a noisy trajectory from the parameters.
     * @param parameters        A matrix [num_dimensions][num_parameters] of the current optimized parameters
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param rollout_number    The index of the noisy trajectory.
     * @param parameters_noise  The parameters + noise
     * @param noise             The noise applied to the parameters
     * @return True if cost were properly computed, otherwise false
     */
    bool generateNoisyParameters(const Eigen::MatrixXd &parameters,
                                 std::size_t start_timestep,
                                 std::size_t num_timesteps,
                                 int iteration_number,
                                 int rollout_number,
                                 Eigen::MatrixXd &parameters_noise,
                                 Eigen::MatrixXd &noise) override
    {
        double rand_noise;
        for (std::size_t d = 0; d < parameters.rows(); d++)
        {
            for (std::size_t t = 0; t < parameters.cols(); t++)
            {
                rand_noise = static_cast<double>(rand() % RAND_MAX) / static_cast<double>(RAND_MAX - 1); // 0 to 1
                rand_noise = 2 * (0.5 - rand_noise);
                noise(d, t) = rand_noise * std_dev_[d];
            }
        }

        parameters_noise = parameters + noise;

        return true;
    }

    /**
     * @brief computes the state costs as a function of the distance from the bias parameters
     * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param costs             A vector containing the state costs per timestep.
     * @param validity          Whether or not the trajectory is valid
     * @return True if cost were properly computed, otherwise false
     */
    bool computeCosts(const Eigen::MatrixXd &parameters,
                      std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      Eigen::VectorXd &costs,
                      bool &validity) override
    {
        if (config_.enableOptVis)
        {
            rate_.sleep();
            visualizer_->delGroup(3);
            visualizer_->setIdGroup(3);
            std::vector<Eigen::Vector3d> traj;
            for (int i = 0; i < num_timesteps; i++)
                traj.emplace_back(parameters.col(i));
            visualizer_->visCurve(traj, ros_visualizer::VisStyle(0.1, 0.7, 0.1, 0.5, 0.01));
        }
        bool ret = computeNoisyCosts(parameters, start_timestep, num_timesteps, iteration_number, -1, costs, validity);
        return ret;
    }

    /**
     * @brief computes the state costs as a function of the distance from the bias parameters
     * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param rollout_number    The index of the noisy trajectory.
     * @param costs             A vector containing the state costs per timestep.
     * @param validity          Whether or not the trajectory is valid
     * @return True if cost were properly computed, otherwise false
     */
    bool computeNoisyCosts(const Eigen::MatrixXd &parameters,
                           std::size_t start_timestep,
                           std::size_t num_timesteps,
                           int iteration_number,
                           int rollout_number,
                           Eigen::VectorXd &costs,
                           bool &validity) override
    {
        costs.setZero(num_timesteps);
        double cost = 0.0;
        validity = true;

        for (std::size_t t = 0u; t < num_timesteps; t++)
        {
            cost = 0;
            validity &= !collision_penalty_.attachPena(parameters.col(t), cost);
            costs(t) = cost;
        }

        return true;
    }

    /**
     * @brief Filters the given parameters which is applied after the update. It could be used for clipping of joint
     * limits or projecting into the null space of the Jacobian.
     *
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param parameters        The optimized parameters
     * @param updates           The updates to the parameters
     * @return                  True if successful, otherwise false
     */
    bool filterParameterUpdates(std::size_t start_timestep,
                                std::size_t num_timesteps,
                                int iteration_number,
                                const Eigen::MatrixXd &parameters,
                                Eigen::MatrixXd &updates) override
    {
        return smoothParameterUpdates(start_timestep, num_timesteps, iteration_number, updates);
    }

    bool filterNoisyParameters(std::size_t start_timestep,
                               std::size_t num_timesteps,
                               int iteration_number,
                               int rollout_number,
                               Eigen::MatrixXd &parameters,
                               bool &filtered) override
    {
        filtered = true;
        parameters.col(0) = p0_;
        parameters.col(num_timesteps - 1) = p1_;
        return true;
    }

protected:
    /**
     * @brief Perform a smooth update given a noisy update
     * @param start_timestep starting timestep
     * @param num_timesteps number of timesteps
     * @param iteration_number number of interations allowed
     * @param updates returned smooth update
     * @return True if successful, otherwise false
     */
    bool smoothParameterUpdates(std::size_t start_timestep,
                                std::size_t num_timesteps,
                                int iteration_number,
                                Eigen::MatrixXd &updates)
    {
        for (auto d = 0u; d < updates.rows(); d++)
        {
            updates.row(d).transpose() = smoothing_M_ * (updates.row(d).transpose());
        }
        updates.col(0) = Eigen::VectorXd::Zero(updates.rows());
        updates.col(num_timesteps - 1) = Eigen::VectorXd::Zero(updates.rows());
        return true;
    }
};

//! [CfgStompTask Inherit]
/** @brief A dummy task for testing STOMP */
class CfgStompTask : public stomp::Task
{

protected:
    std::shared_ptr<ElSpiderAirInterface> robot_interface_;
    std::shared_ptr<GridMapInterface> gridmap_interface_;
    SwingTrajPlannerConfig config_;

    std::vector<double> std_dev_; /**< Standard deviation used for generating noisy parameters */
    Eigen::MatrixXd smoothing_M_; /**< Matrix used for smoothing the trajectory */

    StompLegCollisionPenalty collision_penalty_;
    StompLegLimitPenalty leglimit_penalty_;
    Eigen::Vector3d p0_, p1_; // Cartesian position
    Eigen::Vector3d p0cfg_, p1cfg_;
    pinocchio::SE3 pose0_, pose1_;
    int index_;

    Eigen::Matrix<double, 3, 2> posBd_;

    // Visualizer
    ros::NodeHandle nh_;
    ros::Rate rate_ = ros::Rate(5);
    std::shared_ptr<GCSVisualizer> visualizer_;
    bool enable_vis_ = false;

public:
    CfgStompTask(SwingTrajPlannerConfig config,
                 std::shared_ptr<ElSpiderAirInterface> robot_interface,
                 std::shared_ptr<GridMapInterface> gridmap_interface,
                 std::shared_ptr<GCSVisualizer> visualizer = nullptr)
        : config_(config), robot_interface_(robot_interface), gridmap_interface_(gridmap_interface),
          collision_penalty_(robot_interface, gridmap_interface, visualizer),
          visualizer_(visualizer)
    {
        if (visualizer != nullptr)
            enable_vis_ = true;

        rate_ = ros::Rate(config.optVisRate);

        collision_penalty_.setupParams(config_);
        leglimit_penalty_.setupParams(config_);
        posBd_ << config_.joint1PosMin, config_.joint1PosMax,
            config_.joint2PosMin, config_.joint2PosMax,
            config_.joint3PosMin, config_.joint3PosMax;
        // generate smoothing matrix
        std_dev_ = {config_.stompStdDev1, config_.stompStdDev2, config_.stompStdDev3};
        stomp::generateSmoothingMatrix(config_.stompNumTimesteps, config_.trajTime / (config_.stompNumTimesteps - 1), smoothing_M_);
        srand(time(0));
    };

    void setup(const pinocchio::SE3 &pose0, const pinocchio::SE3 &pose1,
               const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, int index)
    {
        pose0_ = pose0;
        pose1_ = pose1;
        p0_ = p0;
        p1_ = p1;
        p0cfg_ = robot_interface_->IKFast_foot(point_SE3Act(pose0, p0), index);
        p1cfg_ = robot_interface_->IKFast_foot(point_SE3Act(pose1, p1), index);
        index_ = index;
        collision_penalty_.setExcludeBall(p0, p1);
    }

    void setupVis(std::shared_ptr<GCSVisualizer> visualizer)
    {
        if (visualizer != nullptr)
        {
            enable_vis_ = true;
            visualizer_ = visualizer;
            // collision_penalty_.setupVis(visualizer);
        }
    }

    //! [CfgStompTask Inherit]
    /**
     * @brief Generates a noisy trajectory from the parameters.
     * @param parameters        A matrix [num_dimensions][num_parameters] of the current optimized parameters
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param rollout_number    The index of the noisy trajectory.
     * @param parameters_noise  The parameters + noise
     * @param noise             The noise applied to the parameters
     * @return True if cost were properly computed, otherwise false
     */
    bool generateNoisyParameters(const Eigen::MatrixXd &parameters,
                                 std::size_t start_timestep,
                                 std::size_t num_timesteps,
                                 int iteration_number,
                                 int rollout_number,
                                 Eigen::MatrixXd &parameters_noise,
                                 Eigen::MatrixXd &noise) override
    {
        double rand_noise;
        for (std::size_t d = 0; d < parameters.rows(); d++)
        {
            for (std::size_t t = 0; t < parameters.cols(); t++)
            {
                rand_noise = static_cast<double>(rand() % RAND_MAX) / static_cast<double>(RAND_MAX - 1); // 0 to 1
                rand_noise = 2 * (0.5 - rand_noise);
                noise(d, t) = rand_noise * std_dev_[d];
            }
        }

        parameters_noise = parameters + noise;

        return true;
    }

    /**
     * @brief computes the state costs as a function of the distance from the bias parameters
     * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param costs             A vector containing the state costs per timestep.
     * @param validity          Whether or not the trajectory is valid
     * @return True if cost were properly computed, otherwise false
     */
    bool computeCosts(const Eigen::MatrixXd &parameters,
                      std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      Eigen::VectorXd &costs,
                      bool &validity) override
    {
        if (config_.enableOptVis)
        {
            rate_.sleep();
            visualizer_->delGroup(3);
            visualizer_->setIdGroup(3);
            std::vector<Eigen::Vector3d> traj;
            for (int i = 0; i < num_timesteps; i++)
                traj.emplace_back(point_SE3Act(poseLinearInterp(pose0_, pose1_, (double)i / (num_timesteps - 1)).inverse(),
                                               robot_interface_->FK_foot(parameters.col(i), index_)));
            visualizer_->visCurve(traj, ros_visualizer::VisStyle(0.1, 0.7, 0.1, 0.5, 0.01));
        }
        bool ret = computeNoisyCosts(parameters, start_timestep, num_timesteps, iteration_number, -1, costs, validity);
        return ret;
    }

    /**
     * @brief computes the state costs as a function of the distance from the bias parameters
     * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param rollout_number    The index of the noisy trajectory.
     * @param costs             A vector containing the state costs per timestep.
     * @param validity          Whether or not the trajectory is valid
     * @return True if cost were properly computed, otherwise false
     */
    bool computeNoisyCosts(const Eigen::MatrixXd &parameters,
                           std::size_t start_timestep,
                           std::size_t num_timesteps,
                           int iteration_number,
                           int rollout_number,
                           Eigen::VectorXd &costs,
                           bool &validity) override
    {
        costs.setZero(num_timesteps);
        double cost = 0.0;
        validity = true;

        for (std::size_t t = 0u; t < num_timesteps; t++)
        {
            cost = 0;
            validity &= !collision_penalty_.attachPena(poseLinearInterp(pose0_, pose1_, (double)t / (num_timesteps - 1)),
                                                       parameters.col(t), index_, cost);
            // validity &= !leglimit_penalty_.attachPena(parameters.col(t), cost);
            costs(t) = cost;
        }

        return true;
    }

    /**
     * @brief Filters the given parameters which is applied after the update. It could be used for clipping of joint
     * limits or projecting into the null space of the Jacobian.
     *
     * @param start_timestep    The start index into the 'parameters' array, usually 0.
     * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
     * @param iteration_number  The current iteration count in the optimization loop
     * @param parameters        The optimized parameters
     * @param updates           The updates to the parameters
     * @return                  True if successful, otherwise false
     */
    bool filterParameterUpdates(std::size_t start_timestep,
                                std::size_t num_timesteps,
                                int iteration_number,
                                const Eigen::MatrixXd &parameters,
                                Eigen::MatrixXd &updates) override
    {
        return smoothParameterUpdates(start_timestep, num_timesteps, iteration_number, updates);
    }

    bool filterNoisyParameters(std::size_t start_timestep,
                               std::size_t num_timesteps,
                               int iteration_number,
                               int rollout_number,
                               Eigen::MatrixXd &parameters,
                               bool &filtered) override
    {
        // TODO: performance improve
        filtered = true;
        for (int i = 0; i < num_timesteps; i++)
        {
            for (int j = 0; j < parameters.rows(); j++)
            {
                if (parameters(j, i) < posBd_(j, 0))
                    parameters(j, i) = posBd_(j, 0);
                else if (parameters(j, i) > posBd_(j, 1))
                    parameters(j, i) = posBd_(j, 1);
            }
        }
        parameters.col(0) = p0cfg_;
        parameters.col(num_timesteps - 1) = p1cfg_;
        return true;
    }

protected:
    /**
     * @brief Perform a smooth update given a noisy update
     * @param start_timestep starting timestep
     * @param num_timesteps number of timesteps
     * @param iteration_number number of interations allowed
     * @param updates returned smooth update
     * @return True if successful, otherwise false
     */
    bool smoothParameterUpdates(std::size_t start_timestep,
                                std::size_t num_timesteps,
                                int iteration_number,
                                Eigen::MatrixXd &updates)
    {
        for (auto d = 0u; d < updates.rows(); d++)
        {
            updates.row(d).transpose() = smoothing_M_ * (updates.row(d).transpose());
        }
        updates.col(0) = Eigen::VectorXd::Zero(updates.rows());
        updates.col(num_timesteps - 1) = Eigen::VectorXd::Zero(updates.rows());
        return true;
    }
};
