/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-30 16:07:49
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-04-07 09:31:52
 * @FilePath: /dynamic_MCTS/src/hit_spider/include/hit_spider/robot_state_transition/kinematics_constrain.hh
 * @Description:
 */
#include "constrains/util.hh"
// #include "constrains/solve_LP_GLPK.hh"
#include <myDataType.h>
#include <HexapodParameter.h>

namespace Robot_State_Transition
{
    extern const MatrixXX A_Ji_foot; // 单腿foot坐标系下约束fixJi的约束矩阵
    extern const VectorX b_Ji_foot;  // 单腿fixJi坐标系下约束foot的右边值

    extern const MatrixXX A_foot_Ji; // 单腿foot坐标系下约束fixJi的约束矩阵
    extern const VectorX b_foot_Ji;  // 单腿foot坐标系下约束fixJi的右边值

    /**
     * @ : 质心约束摆动腿足端
     * @description: 
     * @param {hexapod_Base_Pose} &base_pose：机体的位姿
     * @return {*}：返回的是对六个足端的约束方程，7行为一组，摆动腿选择自己对应的约束方程即可
     */
    // std::pair<MatrixXX, VectorX> get_kinematics_con_cog_foot(const MDT::Pose &base_pose);
    // 质心约束足端,返回全部腿约束不等式
    void get_kinematics_con_cog_foot(const MDT::Pose &base_pose, MatrixXX *A_output, VectorX *b_output);

    // 质心约束足端,返回单腿约束不等式
    std::pair<MatrixXX, VectorX> get_oneLegKinematics_con_cog_foot(const MDT::Pose &base_pose, int legN);

    // 足端约束机体的质心
    std::pair<MatrixXX, VectorX> get_kinematics_con_foot_cog(const MDT::Pose &base_pose,const std::vector<int> &support_leg, const MatrixX3 &contactPoints, const MatrixX3 &contactNormals);

    // 定义计算射线与凸包交点的函数
    Vector3 findIntersection(const MatrixX3& A, const VectorX& b, const Vector3& pnt, const Vector3& dir);

    
    // 定义不等式约束函数，判断点是否在约束内
    bool isInConvex(const MatrixX3& A, const VectorX& b, const Vector3& pnt);

    // 计算平面法向量的函数
    Vector3 calculatePlaneNormal(const std::vector<Vector3>& pnts);
}