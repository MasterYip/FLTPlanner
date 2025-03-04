/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-31 20:52:57
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-04-06 21:59:34
 * @FilePath: /dynamic_MCTS/src/hit_spider/include/hit_spider/robot_state_transition/dynamic_constrain.hh
 * @Description:Hexapod离散动力学约束头文件
 */
#include "constrains/util.hh"
#include "constrains/solve_LP_GLPK.hh"
#include "pinocchio/parsers/urdf.hpp"

namespace Robot_State_Transition
{
    extern const double Mass;       // 机体质量
    extern const Vector3 MaxTorque; // 电机最大扭矩

    // 获取质心动力学方程约束Df=d
    std::pair<MatrixXX, VectorX> get_dynamic_eq_constrain(const Vector3 &c, const int &num_support_leg, const MatrixX3 &contactPoints);

    // 获取关节扭矩不等式约束Ax<=b
    std::pair<MatrixXX, VectorX> get_jointToque_neq_constrain(const int &num_support_leg, const MatrixX3 &Joints_angles);

    // 当前状态是否满足动力学约束
    bool is_meet_dynamic_con(const Vector3 &c, const int &num_support_leg, const MatrixX3 &contactPoints, const MatrixX3 &Joints_angles);

    bool is_meet_dynamic_con2(const Vector3 &c, const std::vector<int>& supportLegNumList, const MatrixX3 &contactPoints, const MatrixX3 &Joints_angles, const pinocchio::Model& model);

    // 获取关节扭矩不等式约束Ax<=b
    std::pair<MatrixXX, VectorX> get_jointToque_neq_constrain_oneleg(const std::vector<int>& supportLegNumList, const MatrixX3 &Joints_angles, const pinocchio::Model& model);
}
