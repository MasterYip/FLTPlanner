/**
 * @file vmc_controller.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "hexapod_controller/vmc_controller.hpp"

#include <qpOASES.hpp>

pinocchio::SE3 transformToSE3(const geometry_msgs::TransformStamped &tf)
{
    return pinocchio::SE3(Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                                             tf.transform.rotation.y, tf.transform.rotation.z),
                          Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
}

bool odom2SE3_Motion(const nav_msgs::Odometry &odom, pinocchio::SE3 &pos, pinocchio::Motion &vel)
{
    pos = pinocchio::SE3(Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z),
                         Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
    vel = pinocchio::Motion(Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z),
                            Eigen::Vector3d(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
    return true;
}

Task formulateFrictionConeTask(const hex_contact_flag_t contact_flag,
                               const double mu)
{
    int numContacts = 0;
    for (size_t i = 0; i < 6; ++i)
    {
        if (contact_flag[i])
        {
            numContacts++;
        }
    }
    // Equality constraint: a * lambda = b
    matrix_t a(3 * (6 - numContacts), 18);
    a.setZero();
    size_t j = 0;
    for (size_t i = 0; i < 6; ++i)
    {
        if (!contact_flag[i])
        {
            // Non-contact, let lambda (contact force) be 0
            a.block(3 * j++, 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }
    }
    vector_t b(a.rows());
    b.setZero();

    // Inequality constraint: d * lambda <= f
    matrix_t frictionPyramic(5, 3); // clang-format off
    // H-rep of the friction cone
    frictionPyramic << 0, 0, -1,
                      1, 0, -mu,
                      -1, 0, -mu,
                      0, 1, -mu,
                      0,-1, -mu; // clang-format on

    // matrix_t d(5 * numContacts + 3 * (6 - numContacts), 18); // Why adding 3 * (6 - numContacts) here?
    matrix_t d(5 * numContacts, 18);
    d.setZero();
    j = 0;
    for (size_t i = 0; i < 6; ++i)
    {
        if (contact_flag[i])
        {
            d.block(5 * j++, 3 * i, 5, 3) = frictionPyramic;
        }
    }
    vector_t f = Eigen::VectorXd::Zero(d.rows());
    return {a, b, d, f};
}

/**
 * @brief Get the expected torso acceleration
 * @note All quantities are in the WORLD frame (cartesian coordinates)
 * @param[in] com_pose COM pose
 * @param[in] com_vel  COM velocity
 * @param[in] exp_pose Expected pose
 * @param[in] exp_vel  Expected velocity
 * @param[out] exp_acc  Expected acceleration
 * @return true
 * @return false
 */
bool getExpAcc(const pinocchio::SE3 &com_pose,
               const pinocchio::Motion &com_vel,
               const pinocchio::SE3 &exp_pose,
               const pinocchio::Motion &exp_vel,
               const double Kp, const double Kd,
               pinocchio::Motion &exp_acc)
{
    pinocchio::Motion err_dir = pinocchio::log6(com_pose.actInv(exp_pose));
    pinocchio::Motion err_vel = exp_vel - com_vel;
    exp_acc = Kp * err_dir + Kd * err_vel;
    return true;
}

/**
 * @brief Get the expected wrench
 *
 * @param[in] mass
 * @param[in] mass_matrix Mass matrix in BASE frame
 * @param[in] com_pose COM pose in WORLD frame
 * @param[in] com_vel COM velocity in WORLD frame
 * @param[in] exp_acc Expected acceleration in WORLD frame
 * @param[out] exp_wrench Expected wrench in WORLD frame
 * @return true
 * @return false
 */
bool getExpWrench(const double mass,
                  const Eigen::Matrix3d &mass_matrix,
                  const pinocchio::SE3 &com_pose,
                  const pinocchio::Motion &com_vel,
                  const pinocchio::Motion &exp_acc,
                  pinocchio::Force &exp_wrench)
{
    matrix_t mass_mat_world = com_pose.rotation() * mass_matrix * com_pose.rotation().transpose(); // TODO: test it
    // vector_t torque = mass_mat_world * exp_acc.angular() + com_vel.angular().cross(mass_mat_world * com_vel.angular());
    vector_t torque = mass_mat_world * exp_acc.angular();
    exp_wrench = pinocchio::Force(mass * exp_acc.linear(), torque);
    return true;
}

/**
 * @brief
 *
 * @param exp_wrench Expected wrench in WORLD frame
 * @param com_pose   COM pose in WORLD frame
 * @param contact_flag Contact flag
 * @param foot_pos   Foot positions in BASE frame
 * @param grf        Ground reaction forces in BASE frame
 * @return true
 * @return false
 */
bool getGroundReactionForce(const pinocchio::Force &exp_wrench,
                            const pinocchio::SE3 &com_pose,
                            const double mu,
                            const hex_contact_flag_t contact_flag,
                            const std::vector<Eigen::Vector3d> foot_pos,
                            std::vector<Eigen::Vector3d> &grf)
{
    matrix_t Q(6, 18);
    Eigen::Matrix3d crossMx;
    Q.setZero();
    for (size_t i = 0; i < 6; ++i)
    {
        if (contact_flag[i])
        {
            Q.block(0, 3 * i, 3, 3) = matrix_t::Identity(3, 3);
            pinocchio::skew(foot_pos[i], crossMx);
            Q.block(3, 3 * i, 3, 3) = -crossMx;
        }
    }
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = Q.transpose() * Q;
    vector_t g = Q.transpose() * com_pose.actInv(exp_wrench).toVector(); // TODO: Test it
    Task constraints = formulateFrictionConeTask(contact_flag, mu);
    size_t numConstraints = constraints.b_.size() + constraints.f_.size();
    vector_t lbA(numConstraints), ubA(numConstraints);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, 18);
    A << constraints.a_,
        constraints.d_;
    lbA << constraints.b_,                                       // Equality constraints
        -qpOASES::INFTY * vector_t::Ones(constraints.f_.size()); // Inequality constraints
    ubA << constraints.b_,
        constraints.f_;

    auto qpProblem = qpOASES::QProblem(18, numConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    options.enableEqualities = qpOASES::BT_TRUE;
    qpProblem.setOptions(options);
    int nWsr = 20;
    qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
    vector_t qpSol(18);

    qpProblem.getPrimalSolution(qpSol.data());
    for (size_t i = 0; i < 6; ++i)
    {
        if (contact_flag[i])
        {
            grf[i] = qpSol.segment<3>(3 * i);
        }
    }
    return true;
}

VMCController::VMCController(ros::NodeHandle &nh) : tfListener_(tfBuffer_), rosvis_(nh)
{
    cfg_.loadParameters(nh);
    exp_foot_state_sub_ = nh.subscribe(cfg_.exp_foot_state_topic_name, 1, &VMCController::expFootStateCallback, this);
    fdb_foot_state_sub_ = nh.subscribe(cfg_.fdb_foot_state_topic_name, 1, &VMCController::fdbFootStateCallback, this);
    exp_pose_sub_ = nh.subscribe(cfg_.exp_pose_topic_name, 1, &VMCController::expPoseCallback, this);
    fdb_pose_sub_ = nh.subscribe(cfg_.fdb_pose_topic_name, 1, &VMCController::fdbPoseCallback, this);
    foot_cmd_pub_ = nh.advertise<hexapod_controller::FootCmd>(cfg_.footcmd_topic_name, 1);
}

// Callbacks

bool VMCController::fdbPoseLookup()
{
    try
    {
        // FIXME: Make sure it is updated in real-time
        body_state_tf_ = tfBuffer_.lookupTransform(cfg_.world_frame, cfg_.body_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not get body state: %s", ex.what());
        return false;
    }
    fdb_pose_ = transformToSE3(body_state_tf_);
    recv_fdb_pose_ = true;
    return true;
}

void VMCController::fdbPoseCallback(const nav_msgs::Odometry &msg)
{
    recv_fdb_pose_ = odom2SE3_Motion(msg, fdb_pose_, fdb_vel_);
}

void VMCController::expPoseCallback(const nav_msgs::Odometry &msg)
{
    recv_exp_pose_ = odom2SE3_Motion(msg, exp_pose_, exp_vel_);
}

void VMCController::fdbFootStateCallback(const hexapod_controller::FootState &msg)
{
    fdb_foot_state_ = msg;
    recv_fdb_foot_state_ = true;
}

void VMCController::expFootStateCallback(const hexapod_controller::FootState &msg)
{
    exp_foot_state_ = msg;
    recv_exp_foot_state_ = true;
}

void VMCController::controllLoop()
{
    // fdbPoseLookup();
    if (!(recv_exp_pose_ && recv_exp_foot_state_ && recv_fdb_foot_state_ && recv_fdb_pose_))
    {
        ROS_WARN("Not all states are received");
    }
    // Gravity compensation
    // exp_acc.linear().z() += cfg_.gravity;
}

void VMCController::run()
{
    ros::Rate loop_rate(cfg_.loop_rate);
    while (ros::ok())
    {
        controllLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void VMCController::test_getExpAcc()
{
    ros::Rate loop_rate(50);
    double interval = 0.02;
    pinocchio::SE3 target_pose(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
    pinocchio::Motion target_vel(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    ros_visualizer::VisStyle target_style(1, 0.5, 0.5, 1, 0.2);
    // pinocchio::SE3 state_pose(Eigen::Quaterniond(0.707, 0, 0, 0.707), Eigen::Vector3d(1, 1, 0));
    pinocchio::SE3 state_pose = pinocchio::SE3::Random();
    pinocchio::Motion state_vel(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    ros_visualizer::VisStyle state_style(0.5, 1, 0.5, 0.5, 0.2);

    pinocchio::Motion exp_acc;
    getExpAcc(state_pose, state_vel, target_pose, target_vel, cfg_.Kp, cfg_.Kd, exp_acc);
    while (exp_acc.linear().norm() > 1e-2 && exp_acc.angular().norm() > 1e-2)
    {
        getExpAcc(state_pose, state_vel, target_pose, target_vel, cfg_.Kp, cfg_.Kd, exp_acc);
        state_vel.linear() += exp_acc.linear() * interval;
        state_vel.angular() += exp_acc.angular() * interval;
        state_pose.translation() += state_vel.linear() * interval;
        state_pose.rotation() = state_pose.rotation() * pinocchio::exp3(state_vel.angular() * interval);
        rosvis_.delAll();
        pinocchio::SE3::Quaternion quat(target_pose.rotation());
        rosvis_.visCube(target_pose.translation(), quat.coeffs(), target_style); // Target
        quat = pinocchio::SE3::Quaternion(state_pose.rotation());
        rosvis_.visCube(state_pose.translation(), quat.coeffs(), state_style); // State
        ros::spinOnce();
        loop_rate.sleep();
    }
    return;
}