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
#include <math.h>
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
 *
 */
bool getExpAcc(const pinocchio::SE3 &com_pose,
               const pinocchio::Motion &com_vel,
               const pinocchio::SE3 &exp_pose,
               const pinocchio::Motion &exp_vel,
               const double Kp, const double Kd,
               pinocchio::Motion &exp_acc)
{
    pinocchio::Motion err_dir = com_pose.act(pinocchio::log6(com_pose.actInv(exp_pose)));
    pinocchio::Motion err_vel = exp_vel - com_vel;
    // FIXME: Why err_dir.linear is not correct (aMb.act(motion) may not be the one we want)
    err_dir.linear() = exp_pose.translation() - com_pose.translation();
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
    pinocchio::Force exp_wrench_base;                                                   // PROBLEM: It seems not correct to use com_pose.actInv(exp_wrench) here
    exp_wrench_base.linear() = com_pose.rotation().transpose() * exp_wrench.linear();   // Handle the linear part
    exp_wrench_base.angular() = com_pose.rotation().transpose() * exp_wrench.angular(); // Handle the angular part
    vector_t g = -Q.transpose() * exp_wrench_base.toVector();                           // TODO: Test it
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

    bool ret = qpProblem.getPrimalSolution(qpSol.data());
    for (size_t i = 0; i < 6; ++i)
    {
        if (contact_flag[i])
        {
            grf[i] = qpSol.segment<3>(3 * i);
        }
    }
    return ret;
}

VMCController::VMCController(ros::NodeHandle &nh) : tfListener_(tfBuffer_), rosvis_(nh, "base", "visualizer_markers")
{
    cfg_.loadParameters(nh);
    exp_foot_state_sub_ = nh.subscribe(cfg_.exp_foot_state_topic_name, 1, &VMCController::expFootStateCallback, this);
    fdb_foot_state_sub_ = nh.subscribe(cfg_.fdb_foot_state_topic_name, 1, &VMCController::fdbFootStateCallback, this);
    exp_pose_sub_ = nh.subscribe(cfg_.exp_pose_topic_name, 1, &VMCController::expPoseCallback, this);
    fdb_pose_sub_ = nh.subscribe(cfg_.fdb_pose_topic_name, 1, &VMCController::fdbPoseCallback, this);
    foot_cmd_pub_ = nh.advertise<hexapod_controller::FootCmd>(cfg_.footcmd_topic_name, 1);

    // GRF filter
    for (uint i = 0; i < 6; i++)
    {
        pid_grf_.emplace_back(MultiDimPID(3, cfg_.pidgrf_Kp, 0.0, 0.0, cfg_.pidgrf_tau, cfg_.pidgrf_lim, 0.0, cfg_.pidgrf_T));
    }
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
    // FIXME: should not be zero
    fdb_vel_ = pinocchio::Motion::Zero();
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

void VMCController::pubFootCmd(const std::vector<Eigen::Vector3d> &footendpos,
                               const std::vector<Eigen::Vector3d> &footendvel,
                               const std::vector<Eigen::Vector3d> &footendeffort)
{
    hexapod_controller::FootCmd footcmd;
    footcmd.header.stamp = ros::Time::now();
    footcmd.feedforward_type = 0; // Default 0, foot force feedforward
    std::vector<double> joint_kp, joint_kd;
    if (!cfg_.sim) // Hardware
    {

        joint_kp = cfg_.joint_kp;
        joint_kd = cfg_.joint_kd;
    }
    else // Gazebo
    {
        joint_kp = cfg_.joint_kp_sim;
        joint_kd = cfg_.joint_kd_sim;
    }
    for (int i = 0; i < 6; ++i)
    {
        geometry_msgs::Point pt;
        pt.x = footendpos[i][0];
        pt.y = footendpos[i][1];
        pt.z = footendpos[i][2];
        footcmd.foot_position.push_back(pt);
        geometry_msgs::Vector3 vec3;
        footcmd.joint_torque.push_back(vec3); // zero, not used
        vec3.x = footendvel[i][0];
        vec3.y = footendvel[i][1];
        vec3.z = footendvel[i][2];
        footcmd.foot_velocity.push_back(vec3);
        vec3.x = footendeffort[i][0];
        vec3.y = footendeffort[i][1];
        vec3.z = footendeffort[i][2];
        footcmd.foot_effort.push_back(vec3);
        vec3.x = joint_kp[0];
        vec3.y = joint_kp[1];
        vec3.z = joint_kp[2];
        footcmd.joint_kp.push_back(vec3);
        vec3.x = joint_kd[0];
        vec3.y = joint_kd[1];
        vec3.z = joint_kd[2];
        footcmd.joint_kd.push_back(vec3);
    }
    foot_cmd_pub_.publish(footcmd);
}

void VMCController::controllLoop()
{
    if (!cfg_.sim)
        fdbPoseLookup(); // FIXME: this do not consider robot velocity

    if (!(recv_exp_pose_ && recv_exp_foot_state_ && recv_fdb_foot_state_ && recv_fdb_pose_))
    {
        ROS_WARN("Not all states are received: ExpPose %d, ExpFootState %d, FdbFootState %d, FdbPose %d",
                 recv_exp_pose_, recv_exp_foot_state_, recv_fdb_foot_state_, recv_fdb_pose_);
        return;
    }
    pinocchio::Motion exp_acc;
    pinocchio::Force exp_wrench;
    hex_contact_flag_t contact_flag;
    std::vector<Eigen::Vector3d> exp_foot_pos(6);
    std::vector<Eigen::Vector3d> fdb_foot_pos(6);
    std::vector<Eigen::Vector3d> foot_vel(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> foot_effort(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> grf(6);

    getExpAcc(fdb_pose_, fdb_vel_, exp_pose_, exp_vel_, cfg_.Kp, cfg_.Kd, exp_acc);
    // Gravity compensation
    exp_acc.linear().z() += cfg_.gravity;
    getExpWrench(cfg_.mass, cfg_.inertia, fdb_pose_, fdb_vel_, exp_acc, exp_wrench);
    for (size_t i = 0; i < 6; ++i)
    {
        exp_foot_pos.at(i) << exp_foot_state_.position.at(i).x,
            exp_foot_state_.position.at(i).y,
            exp_foot_state_.position.at(i).z;
        contact_flag[i] = exp_foot_state_.contact.at(i);
        fdb_foot_pos.at(i) << fdb_foot_state_.position.at(i).x,
            fdb_foot_state_.position.at(i).y,
            fdb_foot_state_.position.at(i).z;
    }
    getGroundReactionForce(exp_wrench, fdb_pose_, cfg_.mu,
                           contact_flag, fdb_foot_pos, grf);
    for (size_t i = 0; i < 6; ++i)
    {
        // foot_effort.at(i) = -grf.at(i);
        // GRF PID filter
        if (contact_flag[i])
            foot_effort.at(i) = pid_grf_.at(i).update(-grf.at(i), foot_effort.at(i)) * cfg_.pidgrf_T + foot_effort.at(i);
        else
            foot_effort.at(i).setZero();
    }
    // Pub foot_cmd
    pubFootCmd(exp_foot_pos, foot_vel, foot_effort);

    // Visualization
    // NOTE: rosvis is under frame `base`
    double vis_scale = 0.002;
    rosvis_.delAll();
    // Foot
    rosvis_.visSphere(fdb_foot_pos);
    // Body
    rosvis_.visCube(Eigen::Vector3d::Zero(), Eigen::Vector4d(1, 0, 0, 0),
                    ros_visualizer::VisStyle(0.5, 1, 0.5, 0.5, 0.1)); // State
    // GRF
    for (size_t i = 0; i < 6; ++i)
    {
        if (contact_flag[i])
            rosvis_.visArrow(fdb_foot_pos[i], fdb_foot_pos[i] + grf[i] * vis_scale,
                             ros_visualizer::VisStyle(1, 0, 0, 1, 0.01));
    }
    rosvis_.visArrow(Eigen::Vector3d::Zero(), exp_wrench.linear() * vis_scale,
                     ros_visualizer::VisStyle(0, 0, 1, 1, 0.04));
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
    ros::Rate loop_rate(40);
    double interval = 0.025;
    // pinocchio::SE3 target_pose(Eigen::Quaterniond(0.9, 0, 0, 0.1), Eigen::Vector3d(0, 1, 0));
    // pinocchio::SE3 target_pose(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
    pinocchio::SE3 target_pose = pinocchio::SE3::Random();
    pinocchio::Motion target_vel(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    ros_visualizer::VisStyle target_style(1, 0.5, 0.5, 1, 0.2);
    // pinocchio::SE3 state_pose(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(1, 0, 0));
    // pinocchio::SE3 state_pose(Eigen::Quaterniond(0.707, 0, 0, 0.707), Eigen::Vector3d(1, 0, 0));
    pinocchio::SE3 state_pose = pinocchio::SE3::Random();
    pinocchio::Motion state_vel(Eigen::Vector3d(0, 3, 0), Eigen::Vector3d(0, 0, 0));
    ros_visualizer::VisStyle state_style(0.5, 1, 0.5, 0.5, 0.2);

    pinocchio::Motion exp_acc;
    getExpAcc(state_pose, state_vel, target_pose, target_vel, cfg_.Kp, cfg_.Kd, exp_acc);
    while (exp_acc.linear().norm() > 1e-1 || exp_acc.angular().norm() > 1e-1)
    {
        getExpAcc(state_pose, state_vel, target_pose, target_vel, cfg_.Kp, cfg_.Kd, exp_acc);
        state_vel.linear() += exp_acc.linear() * interval;
        state_vel.angular() += exp_acc.angular() * interval;
        state_pose.translation() += state_vel.linear() * interval;
        state_pose.rotation() = pinocchio::exp3(state_vel.angular() * interval) * state_pose.rotation();
        // Visualize
        rosvis_.delAll();
        Eigen::Vector4d quat_vec;
        pinocchio::SE3::Quaternion quat(target_pose.rotation()); // This is xyzw
        quat_vec << quat.coeffs().w(), quat.coeffs().x(), quat.coeffs().y(), quat.coeffs().z();
        rosvis_.visCube(target_pose.translation(), quat_vec, target_style); // Target
        quat = pinocchio::SE3::Quaternion(state_pose.rotation());
        quat_vec << quat.coeffs().w(), quat.coeffs().x(), quat.coeffs().y(), quat.coeffs().z();
        rosvis_.visCube(state_pose.translation(), quat_vec, state_style); // State
        rosvis_.visArrow(state_pose.translation(), state_pose.translation() + state_vel.linear() * 0.2);
        rosvis_.visArrow(state_pose.translation(), state_pose.translation() + exp_acc.linear() * 0.04, ros_visualizer::VisStyle(1, 0, 0, 1, 0.04));
        rosvis_.visCurve({state_pose.translation(), target_pose.translation()});
        ros::spinOnce();
        loop_rate.sleep();
    }
    return;
}

void VMCController::test_getGrf()
{
    ros::Rate loop_rate(50);
    double time = 0;
    double interval = 0.02;
    // pinocchio::Motion exp_acc(Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(0, 0, 0));
    pinocchio::Force exp_wrench;
    pinocchio::SE3 state_pose(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
    // Contact flag
    hex_contact_flag_t contact_flag;
    contact_flag.fill(true);
    // Foot pos
    std::vector<Eigen::Vector3d> foot_pos;
    foot_pos.emplace_back(Eigen::Vector3d(0.3, -0.15, 0));
    foot_pos.emplace_back(Eigen::Vector3d(0.0, -0.15, 0));
    foot_pos.emplace_back(Eigen::Vector3d(-0.3, -0.15, 0));
    foot_pos.emplace_back(Eigen::Vector3d(0.3, 0.15, 0));
    foot_pos.emplace_back(Eigen::Vector3d(0.0, 0.15, 0));
    foot_pos.emplace_back(Eigen::Vector3d(-0.3, 0.15, 0));
    std::vector<Eigen::Vector3d> grf(6);

    while (ros::ok())
    {
        time += interval;
        exp_wrench = pinocchio::Force(Eigen::Vector3d(sin(time) * 0.2, cos(time) * 0.2, 1), Eigen::Vector3d(0, 0, 0.3));
        getGroundReactionForce(exp_wrench, state_pose, cfg_.mu, contact_flag, foot_pos, grf);
        rosvis_.delAll();
        // Foot
        rosvis_.visSphere(foot_pos);
        // Body
        Eigen::Vector4d quat_vec;
        pinocchio::SE3::Quaternion quat(state_pose.rotation()); // This is xyzw
        quat_vec << quat.coeffs().w(), quat.coeffs().x(), quat.coeffs().y(), quat.coeffs().z();
        rosvis_.visCube(state_pose.translation(), quat_vec, ros_visualizer::VisStyle(0.5, 1, 0.5, 0.5, 0.1)); // State
        // GRF
        for (size_t i = 0; i < 6; ++i)
        {
            rosvis_.visArrow(foot_pos[i], foot_pos[i] + grf[i], ros_visualizer::VisStyle(1, 0, 0, 1, 0.01));
        }
        rosvis_.visArrow(state_pose.translation(), state_pose.translation() + exp_wrench.linear(), ros_visualizer::VisStyle(0, 0, 1, 1, 0.04));
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void VMCController::test_fdbCalcGrf()
{
    ros::Rate loop_rate(cfg_.loop_rate);
    // ros::Rate loop_rate(10);
    // while (!(recv_exp_pose_ && recv_exp_foot_state_ && recv_fdb_foot_state_ && recv_fdb_pose_))
    while (!(recv_fdb_foot_state_ && recv_fdb_pose_))
    {
        ROS_WARN("Not all states are received");
        ros::spinOnce();
        loop_rate.sleep();
    }
    // Expeccted
    // exp_pose_ = pinocchio::SE3::Identity();
    exp_pose_ = fdb_pose_;              // Keep the same
    exp_pose_.translation().z() = 0.24; // Nominal height
    exp_vel_ = pinocchio::Motion::Zero();
    // Vars
    pinocchio::Motion exp_acc;
    pinocchio::Force exp_wrench;
    hex_contact_flag_t contact_flag;
    contact_flag.fill(true);
    std::vector<Eigen::Vector3d> grf(6);
    std::vector<Eigen::Vector3d> foot_pos(6);
    std::vector<Eigen::Vector3d> foot_vel(6, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> foot_effort(6);
    while (ros::ok())
    {
        // exp_pose_.translation().z() = 0.24;
        // exp_pose_.translation().x() =
        getExpAcc(fdb_pose_, fdb_vel_, exp_pose_, exp_vel_, cfg_.Kp, cfg_.Kd, exp_acc);
        exp_acc.linear().z() += cfg_.gravity;
        getExpWrench(cfg_.mass, cfg_.inertia, fdb_pose_, fdb_vel_, exp_acc, exp_wrench);
        std::cout << "Exp Wrench:\n"
                  << exp_wrench << std::endl;
        for (size_t i = 0; i < 6; ++i)
        {
            foot_pos.at(i) << fdb_foot_state_.position.at(i).x,
                fdb_foot_state_.position.at(i).y,
                fdb_foot_state_.position.at(i).z;
        }
        getGroundReactionForce(exp_wrench, fdb_pose_, cfg_.mu,
                               contact_flag, foot_pos, grf);
        for (size_t i = 0; i < 6; ++i)
        {
            foot_effort.at(i) = -grf.at(i);
        }
        // Pub foot_cmd
        pubFootCmd(foot_pos, foot_vel, foot_effort);

        // Visualization
        // NOTE: rosvis is under frame `base`
        double vis_scale = 0.002;
        rosvis_.delAll();
        // Foot
        rosvis_.visSphere(foot_pos);
        // Body
        // Eigen::Vector4d quat_vec;
        // pinocchio::SE3::Quaternion quat(fdb_pose_.rotation()); // This is xyzw
        // quat_vec << quat.coeffs().w(), quat.coeffs().x(), quat.coeffs().y(), quat.coeffs().z();
        rosvis_.visCube(Eigen::Vector3d::Zero(), Eigen::Vector4d(1, 0, 0, 0),
                        ros_visualizer::VisStyle(0.5, 1, 0.5, 0.5, 0.1)); // State
        // GRF
        for (size_t i = 0; i < 6; ++i)
        {
            rosvis_.visArrow(foot_pos[i], foot_pos[i] + grf[i] * vis_scale,
                             ros_visualizer::VisStyle(1, 0, 0, 1, 0.01));
        }
        rosvis_.visArrow(Eigen::Vector3d::Zero(), exp_wrench.linear() * vis_scale,
                         ros_visualizer::VisStyle(0, 0, 1, 1, 0.04));
        ros::spinOnce();
        loop_rate.sleep();
    }
}