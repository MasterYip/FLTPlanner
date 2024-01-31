/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-31 20:53:25
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-04-13 14:11:45
 * @FilePath: /w3_Bretl/src/hit_spider/src/robot_state_transition/dynamic_constrain.cpp
 * @Description: Hexapod离散动力学约束源文件
 */
#include "constrains/dynamic_constrain.hh"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/model/forward-kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <user.h>

namespace Robot_State_Transition
{
    namespace
    {
        Vector3 init_max_torque()
        {
            Vector3 max_T;
            max_T(0) = 0.286 * 2240; // 根关节扭矩：电机额定转矩 * 减速比 = 640.64Nm
            max_T(1) = 0.747 * 2240; // 根关节扭矩：电机额定转矩 * 减速比 = 1673.28Nm  //  0.1 * 2240
            max_T(2) = 0.286 * 4640; // 根关节扭矩：电机额定转矩 * 减速比 = 1327.04Nm  // 0.02 * 4640

            return max_T;
        }

        /**
         * @ : 获取雅克比矩阵
         * @description:
         * @param {double} &t：关节角度构成的向量
         * @return {*}
         */
        Eigen::Matrix<double, 3, 3> get_Jacobian(const Vector3 &t)
        {
            Eigen::Matrix<double, 3, 3> J;
            const double t1 = t(0);
            const double t2 = t(1);
            const double t3 = t(2);

            J << -sin(t1) * (sin(t2 + t3) / (double)2.0 + cos(t2) / (double)2.0 + 0.18),
                cos(t1) * (cos(t2 + t3) / (double)2.0 - sin(t2) / (double)2.0), (cos(t2 + t3) * cos(t1)) / (double)2.0,
                cos(t1) * (sin(t2 + t3) / (double)2.0 + cos(t2) / (double)2.0 + 0.18), sin(t1) * (cos(t2 + t3) / (double)2.0 - sin(t2) / (double)2.0), (cos(t2 + t3) * sin(t1)) / (double)2.0,
                0, sin(t2 + t3) / (double)2.0 + cos(t2) / (double)2.0, sin(t2 + t3) / (double)2.0;

            return J;
        }

        /**
         * @ : 单腿动力学中的重力加速度项
         * @description:
         * @param {Vector3} &t：关节角度
         * @return {*}
         */
        Vector3 get_legMss_consume_Torque(const Vector3 &t)
        {
            Vector3 Leg_T;
            // const double theta1 = t(0);
            const double theta2 = t(1);
            const double theta3 = t(2);

            Leg_T(0) = 0;
            Leg_T(1) = (629153.0 * 9.81 * cos(theta2 + theta3)) / (double)62500.0 + (37009.0 * 9.81 * sin(theta2 + theta3)) / (double)20000.0 - (869173.0 * 9.81 * cos(theta2)) / (double)1000000.0;
            Leg_T(2) = (37009.0 * 9.81 * sin(theta2 + theta3)) / (double)20000.0;

            return Leg_T;
        }

    }

    // const double Mass = 10.8957;
    const double Mass = 200;
    const Vector3 MaxTorque = init_max_torque();



    std::pair<MatrixXX, VectorX> get_dynamic_eq_constrain(const Vector3 &c, const int &num_support_leg, const MatrixX3 &contactPoints)
    {
        assert(num_support_leg == contactPoints.rows());
        MatrixXX D(6, 3 * num_support_leg);
        VectorX d(6);

        // 对D赋值
        for (int i = 0; i < num_support_leg; ++i)
        {
            D.block(0, 3 * i, 3, 3).setIdentity();                                   // 单位矩阵
            D.block(3, 3 * i, 3, 3) = crossMatrix(contactPoints.row(i).transpose()); // 足端位置构成的反对称矩阵
        }

        // 对d赋值
        d.setZero();
        d(2) = 9.81 * Mass;        // 重力平衡
        d(3) = c(1) * 9.81 * Mass; // 重力构成的倾覆力矩平衡
        d(4) = -1 * c(0) * 9.81 * Mass;

        return std::make_pair(D, d);
    }

    std::pair<MatrixXX, VectorX> get_jointToque_neq_constrain(const int &num_support_leg, const MatrixX3 &Joints_angles)
    {
        assert(num_support_leg == Joints_angles.rows());

        MatrixXX A;
        VectorX b;

        A.resize(6 * num_support_leg, 3 * num_support_leg);
        b.resize(6 * num_support_leg);
        A.setZero();
        b.setZero();

        for (int i = 0; i < num_support_leg; ++i)
        {
            Eigen::Matrix<double, 3, 3> J = get_Jacobian(Joints_angles.row(i).transpose());
            Vector3 Leg_T = get_legMss_consume_Torque(Joints_angles.row(i).transpose());

            A.block(6 * i, 3 * i, 3, 3) = J.transpose();      // J^T <= maxT - Leg_T
            A.block(6 * i + 3, 3 * i, 3, 3) = -J.transpose(); //-J^T <= maxT + Leg_T

            b.block(6 * i, 0, 3, 1) = MaxTorque - Leg_T;     // J^T <= maxT - Leg_T
            b.block(6 * i + 3, 0, 3, 1) = MaxTorque + Leg_T; //-J^T <= maxT + Leg_T
        }

        return std::make_pair(A, b);
    }

    int getLegOrder(const std::string& legName)
    {
        if (legName == "LB_FOOT") {
            return 0;
        } else if (legName == "LF_FOOT") {
            return 3;
        } else if (legName == "LM_FOOT") {
            return 6;
        } else if (legName == "RB_FOOT") {
            return 9;
        } else if (legName == "RF_FOOT") {
            return 12;
        } else if (legName == "RM_FOOT") {
            return 15;
        } else {
            std::cout << "wrong name" << std::endl;
            exit(0);
        }
    }

    int getLegOrderFromLegNum(const int& legN)
    {
        if (legN == 5) {
            return 0;
        } else if (legN == 3) {
            return 3;
        } else if (legN == 4) {
            return 6;
        } else if (legN == 2) {
            return 9;
        } else if (legN == 0) {
            return 12;
        } else if (legN == 1) {
            return 15;
        } else {
            std::cout << "wrong name" << std::endl;
            exit(0);
        }
    }

    std::string getFootNameFromLegNum(const int& legN)
    {
        if (legN == 5) {
            return "LB_FOOT";
        } else if (legN == 3) {
            return "LF_FOOT";
        } else if (legN == 4) {
            return "LM_FOOT";
        } else if (legN == 2) {
            return "RB_FOOT";
        } else if (legN == 0) {
            return "RF_FOOT";
        } else if (legN == 1) {
            return "RM_FOOT";
        } else {
            std::cout << "wrong name" << std::endl;
            exit(0);
        }
    }


    std::pair<MatrixXX, VectorX> get_jointToque_neq_constrain_oneleg(const std::vector<int>& supportLegNumList, const MatrixX3 &Joints_angles, const pinocchio::Model& model)
    {

        Vector3 max_T;
        max_T(0) = 35.0; // 根关节扭矩
        max_T(1) = 35.0; // 根关节扭矩
        max_T(2) = 35.0; // 根关节扭矩
        int num_support_leg = supportLegNumList.size();

        MatrixXX A;
        VectorX b;

        A.resize(6 * num_support_leg, 3 * num_support_leg);
        b.resize(6 * num_support_leg);
        A.setZero();
        b.setZero();


        // this part can be initialized in the user.h file

        // URDF 文件路径
        // std::string urdfPath = "/home/oem/aConstrainPlan/catkin_viewURDF/src/ROS-HexMini-Visual/el_mini/urdf/el_mini.urdf";

        // // 从 URDF 文件构建模型
        // pinocchio::Model model;
        // pinocchio::urdf::buildModel(urdfPath, model);
        // 创建数据结构
        // pinocchio::Data data(model);
        pinocchio::Data data = USER::HexMini_PinoModel_Data; /// 这里

        // 初始化关节位置、速度和加速度
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
        // q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
        for (int i = 0; i < num_support_leg; ++i)
        {
            int footOrder = getLegOrderFromLegNum(supportLegNumList[i]-1); 
            q.segment(footOrder, 3) = Joints_angles.row(i);
        }


        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

        // 计算重力compensate
        Eigen::VectorXd g_grav = pinocchio::rnea(model, data, q, v, a);

        for(int i =0; i< supportLegNumList.size(); i++)
        {
            int legNum = supportLegNumList[i]-1;
            int footOrder = getLegOrderFromLegNum(legNum);  // this is the start position for Jacobian Matrix or g_grav term, which is decided by the urdf file.
            // 计算雅可比矩阵
            Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianM(6, model.nv);
            pinocchio::computeFrameJacobian(model, data, q, model.getFrameId(getFootNameFromLegNum(legNum)), pinocchio::LOCAL_WORLD_ALIGNED, jacobianM);
            Eigen::Matrix<double, 3, 3> Jc_LB_foot = jacobianM.topRows<3>().middleCols<3>(footOrder);


            Vector3 grav_term;
            grav_term << g_grav(footOrder), g_grav(footOrder+1), g_grav(footOrder+2);


            A.block(6 * i, 3 * i, 3, 3) = Jc_LB_foot.transpose();      // J^T * f <= grav(q) + touqueMax
            A.block(6 * i + 3, 3 * i, 3, 3) = -Jc_LB_foot.transpose();  // -J^T * f <= -grav(q) + touqueMax

            b.block(6 * i, 0, 3, 1) = max_T + grav_term;     // J^T * f <= grav(q) + touqueMax
            b.block(6 * i + 3, 0, 3, 1) = max_T - grav_term; // -J^T * f <= -grav(q) + touqueMax

        }


        // AA * f <= bb
        return std::make_pair(A, b);
    }










    /**
     * @ : 判断是否满足力矩约束
     * @description:
     * @param c: 质心位置
     * @param num_support_leg: 支撑腿数
     * @param contactPoints: 接触点位置
     * @param Joints_angles: 关节角度
     * 
    */
    bool is_meet_dynamic_con2(const Vector3 &c, const std::vector<int>& supportLegNumList, const MatrixX3 &contactPoints, const MatrixX3 &Joints_angles, const pinocchio::Model& model)
    {
        auto Dd = get_dynamic_eq_constrain(c, supportLegNumList.size(), contactPoints);
        auto Ab = get_jointToque_neq_constrain_oneleg(supportLegNumList, Joints_angles, model);
        // auto Ab = get_jointToque_neq_constrain(supportLegNumList.size(), Joints_angles);

        VectorX g, minBound, maxBound;

        bool flag = solve_LP_GLPK(Ab.first, Ab.second, Dd.first, Dd.second, g, minBound, maxBound);

        return flag;
    }







    /**
     * @ : 判断是否满足力矩约束
     * @description:
     * @param c: 质心位置
     * @param num_support_leg: 支撑腿数
     * @param contactPoints: 接触点位置
     * @param Joints_angles: 关节角度
     * 
    */
    bool is_meet_dynamic_con(const Vector3 &c, const int &num_support_leg, const MatrixX3 &contactPoints, const MatrixX3 &Joints_angles)
    {
        auto Dd = get_dynamic_eq_constrain(c, num_support_leg, contactPoints);
        auto Ab = get_jointToque_neq_constrain(num_support_leg, Joints_angles);

        VectorX g, minBound, maxBound;

        bool flag = solve_LP_GLPK(Ab.first, Ab.second, Dd.first, Dd.second, g, minBound, maxBound);
        // std::cout << "g: " << std::endl;
        return flag;
    }

}
