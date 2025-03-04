/**
 * 
 * @file hexapodRobot.cpp
 * @brief 
 * @version 0.1
 * @date 2023-05-18
 * @author xp
 * 
*/


#include <HexapodParameter.h>


namespace HexapodParameter
{
    std::vector<MDT::Vector6b> createInitialSupportList(void);


    Eigen::Isometry3d init_TransMatrix_Body_2_fixJi_HexMini(float x, float y, float z);
    Eigen::Isometry3d init_TransMatrix_FixJi_Body_HexMini(float x, float y, float z);

   

    const Eigen::Isometry3d TransMatrix_FixJi_Body_forHexMini[6] = {init_TransMatrix_FixJi_Body_HexMini(0.3,-0.08,0.01), init_TransMatrix_FixJi_Body_HexMini(0.0,-0.14,0.01), init_TransMatrix_FixJi_Body_HexMini(-0.3,-0.08,0.01),
                                                        init_TransMatrix_FixJi_Body_HexMini(0.3,0.08,0.01), init_TransMatrix_FixJi_Body_HexMini(0.0,0.14,0.01), init_TransMatrix_FixJi_Body_HexMini(-0.3,0.08,0.01)}; 

    const Eigen::Isometry3d TransMatrix_Body_2_fixJi_forHexMini[6] = {init_TransMatrix_Body_2_fixJi_HexMini(0.3,-0.08,0.01), init_TransMatrix_Body_2_fixJi_HexMini(0.0,-0.14,0.01), init_TransMatrix_Body_2_fixJi_HexMini(-0.3,-0.08,0.01),
                                                        init_TransMatrix_Body_2_fixJi_HexMini(0.3,0.08,0.01), init_TransMatrix_Body_2_fixJi_HexMini(0.0,0.14,0.01), init_TransMatrix_Body_2_fixJi_HexMini(-0.3,0.08,0.01)}; 


    const std::vector<MDT::Vector6b> initialSupportList = createInitialSupportList(); 


    /***************************************************************************************************/


    std::vector<MDT::Vector6b> createInitialSupportList(void)
    {
        std::vector<MDT::Vector6b> initialSupportList;
        for(int leg1=0; leg1<2; leg1++)
        {
            for(int leg2=0; leg2<2; leg2++)
            {
                for(int leg3=0; leg3<2; leg3++)
                {
                    for(int leg4=0; leg4<2; leg4++)
                    {
                        for(int leg5=0; leg5<2; leg5++)
                        {
                            for(int leg6=0; leg6<2; leg6++)
                            {
                                if(leg1+leg2+leg3+leg4+leg5+leg6 > 2)  // 这里 1 代表支撑
                                {
                                    if(leg1+leg2+leg3+leg4+leg5+leg6 == 6)  // 这里 1 代表支撑
                                    {
                                        continue;
                                    }
                                    MDT::Vector6b state;

                                    int legs[] = {leg1, leg2, leg3, leg4, leg5, leg6};

                                    for (int i = 0; i < 6; ++i) {
                                        state[i] = (legs[i] == 1) ? MDT::SUPPORT_FLAG : MDT::SWING_FLAG;
                                    }
                                    initialSupportList.push_back(state);
                                }
                            }
                        }
                    }
                }
            }
        }

        return initialSupportList;
    }


    Eigen::Isometry3d init_TransMatrix_FixJi_Body_HexMini(float x, float y, float z)
    {
        Eigen::Isometry3d T;

        T.matrix().row(0) << 1, 0, 0, x; 
        T.matrix().row(1) << 0, 1, 0, y;
        T.matrix().row(2) << 0, 0, 1, z;
        T.matrix().row(3) << 0, 0, 0, 1;

        T = T.inverse();
        return T;
    }

    Eigen::Isometry3d init_TransMatrix_Body_2_fixJi_HexMini(float x, float y, float z)
    {
        Eigen::Isometry3d T;

        T.matrix().row(0) << 1, 0, 0, x; 
        T.matrix().row(1) << 0, 1, 0, y;
        T.matrix().row(2) << 0, 0, 1, z;
        T.matrix().row(3) << 0, 0, 0, 1;

        return T;
    }



    // 初始化机器人状态,并赋初值
    MDT::RobotState initRobotState(const MDT::Pose &robotPoseW,  MDT::Vector6b gaitToNow, float moveDirection, const Parameters &params)
    {
        MDT::RobotState state_;
        state_.initialize();
        state_.pose = robotPoseW;
                state_.faultStateToNow << MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG;
        for(int i=0;i<6;i++)
        {
            state_.feetPosition[i] = MDT::pointRotationAndTrans(params.norminalFoothold_B[i], robotPoseW.getT_W_B());
                        state_.feetNormalVector[i] << 0, 0, 1; // 默认法向量竖直向上
            state_.gaitToNow[i] = gaitToNow[i];
            state_.maxNormalForce[i] = 1000.0f;
            state_.frcitionMu[i] = 0.8f;
        }
        state_.moveDirection = moveDirection;
        
        return state_;
    }



}