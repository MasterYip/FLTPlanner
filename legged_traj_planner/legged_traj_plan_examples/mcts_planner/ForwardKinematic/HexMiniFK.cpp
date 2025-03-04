
#include "HexMiniFK.h"


// 求解全身的正运动学
// q_MCTS_planning_order: 输入顺序为MCTS规划顺序，即RF, RM, RB, LF, LM, LB
Eigen::Matrix<double, 6, 3> wholeBodyFK(Eigen::VectorXd q_MCTS_planning_order, Parameters &params)
{
    Eigen::Matrix<double, 6, 3> feetPosition_B;
    // 更改为pinocchio的顺序接口
    Eigen::VectorXd q = Eigen::VectorXd::Zero(18);
    q.segment(12, 3) = q_MCTS_planning_order.segment(0, 3);
    q.segment(15, 3) = q_MCTS_planning_order.segment(3, 3);
    q.segment(9, 3) = q_MCTS_planning_order.segment(6, 3);
    q.segment(3, 3) = q_MCTS_planning_order.segment(9, 3);
    q.segment(6, 3) = q_MCTS_planning_order.segment(12, 3);
    q.segment(0, 3) = q_MCTS_planning_order.segment(15, 3);

    pinocchio::forwardKinematics(params.HexMini_PinoModel, params.HexMini_PinoModel_Data, q);
    pinocchio::updateFramePlacements(params.HexMini_PinoModel, params.HexMini_PinoModel_Data);
    for(int i=0; i<6; i++)
    {
        auto footP = params.HexMini_PinoModel_Data.oMf[params.HexMini_PinoModel.getFrameId(params.FootName[i])].translation();
        feetPosition_B.row(i) = footP;
    }

    return feetPosition_B;
}

