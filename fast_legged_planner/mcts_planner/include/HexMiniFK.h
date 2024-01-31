#ifndef ROBOT_STATE_TRANSITION_BRETL_HH
#define ROBOT_STATE_TRANSITION_BRETL_HH

#include "pinocchio/parsers/urdf.hpp"
#include "user.h"

    /**
     * 机器人正运动学 - 注意顺序为RF, RM, RB, LF, LM, LB
     * @param q_MCTS_planning_order 机器人关节角度 
     * @return 机器人六条腿的末端位置 - 注意参考坐标系为机体坐标系
    */
    Eigen::Matrix<double, 6, 3> wholeBodyFK(Eigen::VectorXd q_MCTS_planning_order);

#endif