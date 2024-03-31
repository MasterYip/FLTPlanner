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
    std::array<std::array<MDT::POINT, 4>, 6> init_workspaceVertex(
            const std::vector<MDT::Pose>& transList, double rectangleLength, double rectangleWidth);
    // Eigen::Isometry3d init_TransMatrix_FixJi_Body(const int &num);
    // Eigen::Isometry3d init_TransMatrix_FixGu_Body(const int &num);
    bool isInLegsWorkspace(const int &legNum_, const MDT::Pose &robotPose, const MDT::POINT &pointW);
    MDT::VecVector3d setLegsWorkspacePntCloud(const int &leg_num);
    MDT::POINT init_defaultFoothold(const int &num);
    Eigen::Isometry3d init_TransMatrix_Body_2_fixJi_HexMini(float x, float y, float z);
    Eigen::Isometry3d init_TransMatrix_FixJi_Body_HexMini(float x, float y, float z);

    const float body_fixJi_theta[6] = {_PI_ * 2 / 3, _PI_, _PI_ * 4 / 3, _PI_ * 5 / 3, 0, _PI_ * 1 / 3};

   
    // const Eigen::Isometry3d TransMatrix_FixJi_Body[6] = {init_TransMatrix_FixJi_Body(0), init_TransMatrix_FixJi_Body(1), init_TransMatrix_FixJi_Body(2),
    //                                                     init_TransMatrix_FixJi_Body(3), init_TransMatrix_FixJi_Body(4), init_TransMatrix_FixJi_Body(5)};
    // const Eigen::Isometry3d TransMatrix_FixGu_Body[6] = {init_TransMatrix_FixGu_Body(0), init_TransMatrix_FixGu_Body(1), init_TransMatrix_FixGu_Body(2),
    //                                                     init_TransMatrix_FixGu_Body(3), init_TransMatrix_FixGu_Body(4), init_TransMatrix_FixGu_Body(5)};

    const Eigen::Isometry3d TransMatrix_FixJi_Body_forHexMini[6] = {init_TransMatrix_FixJi_Body_HexMini(0.3,-0.08,0.01), init_TransMatrix_FixJi_Body_HexMini(0.0,-0.14,0.01), init_TransMatrix_FixJi_Body_HexMini(-0.3,-0.08,0.01),
                                                        init_TransMatrix_FixJi_Body_HexMini(0.3,0.08,0.01), init_TransMatrix_FixJi_Body_HexMini(0.0,0.14,0.01), init_TransMatrix_FixJi_Body_HexMini(-0.3,0.08,0.01)}; 

    const Eigen::Isometry3d TransMatrix_Body_2_fixJi_forHexMini[6] = {init_TransMatrix_Body_2_fixJi_HexMini(0.3,-0.08,0.01), init_TransMatrix_Body_2_fixJi_HexMini(0.0,-0.14,0.01), init_TransMatrix_Body_2_fixJi_HexMini(-0.3,-0.08,0.01),
                                                        init_TransMatrix_Body_2_fixJi_HexMini(0.3,0.08,0.01), init_TransMatrix_Body_2_fixJi_HexMini(0.0,0.14,0.01), init_TransMatrix_Body_2_fixJi_HexMini(-0.3,0.08,0.01)}; 


    const std::vector<MDT::Vector6b> initialSupportList = createInitialSupportList(); 




    const MDT::POINT norminalFoothold_B[6] = {init_defaultFoothold(0), init_defaultFoothold(1), init_defaultFoothold(2),
                                                init_defaultFoothold(3), init_defaultFoothold(4), init_defaultFoothold(5)};

    /***************************************************************************************************/
    std::array<std::array<MDT::POINT, 4>, 6> init_workspaceVertex(const std::vector<MDT::Pose>& transList, double rectangleLength, double rectangleWidth)
    {
        std::array<std::array<MDT::POINT, 4>, 6> vertex;

        for (int i = 0; i < 6; i++) {
            double x = transList[i].x;
            double y = transList[i].y;

            // 计算四个顶点的坐标
            vertex[i] = {{
                {x + rectangleLength, y + rectangleWidth, 0},
                {x + rectangleLength, y - rectangleWidth, 0},
                {x - rectangleLength, y - rectangleWidth, 0},
                {x - rectangleLength, y + rectangleWidth, 0}
                
            }};
        }

        return vertex;
    }

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


 // 初始化固定的坐标变换FixJi
    Eigen::Isometry3d init_TransMatrix_FixJi_Body(const int &num)
    {
        Eigen::Isometry3d T;

        T.matrix().row(0) << cos(body_fixJi_theta[num]), sin(body_fixJi_theta[num]), 0, -0.4; //-R
        T.matrix().row(1) << -sin(body_fixJi_theta[num]), cos(body_fixJi_theta[num]), 0, 0;
        T.matrix().row(2) << 0, 0, 1, 0;
        T.matrix().row(3) << 0, 0, 0, 1;

        return T;
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

    // 初始化固定的坐标变换FixGu
    Eigen::Isometry3d init_TransMatrix_FixGu_Body(const int &num)
    {
        Eigen::Isometry3d T1 = init_TransMatrix_FixJi_Body(num);
        Eigen::Isometry3d T;

        T.matrix().row(0) << cos(-_PI_ / 2), sin(-_PI_ / 2), 0, -0.18 * cos(-_PI_ / 2);
        T.matrix().row(1) << -sin(-_PI_ / 2), cos(-_PI_ / 2), 0, 0.18 * sin(-_PI_ / 2);
        T.matrix().row(2) << 0, 0, 1, 0;
        T.matrix().row(3) << 0, 0, 0, 1;
        T = T * T1;

        return T;
    }

    bool isInLegsWorkspace(const int &legNum_, const MDT::Pose &robotPose, const MDT::POINT &pointW)
    {
        // 计算足端目标机器人坐标系到世界坐标系的旋转平移矩阵T
        Eigen::Isometry3d T_W_B = robotPose.getT_W_B();
        Eigen::Vector3d p1_ = pointW;

        // 计算pointW在固定基节坐标系（扇形坐标系）下的坐标

        auto TransMatrix_FixGu_Body =  init_TransMatrix_FixGu_Body(legNum_ - 1);
        Eigen::Vector3d foot_FixGu = TransMatrix_FixGu_Body * T_W_B.inverse() * p1_;

        // 扇形坐标系下计算
        float angle_ = atan2(foot_FixGu.y(), foot_FixGu.x());
        float distance = sqrt(foot_FixGu.x() * foot_FixGu.x() + foot_FixGu.y() * foot_FixGu.y());

        if (distance < USER::LegWorkspaceR && angle_ > _PI_ * 1.0f / 4.0f && angle_ < _PI_ * 3.0f / 4.0f)
        {
            return true;
        }
        return false;
    }
    

    // ------扇形落足点------
    MDT::VecVector3d setLegsWorkspacePntCloud(const int &leg_num)
    {
        static grid_map::GridMap mapData; // 临时变量而已

        MDT::VecVector3d Cloud_p;

        mapData.setGeometry(grid_map::Length(5, 5), 0.1);

        grid_map::Position tmpP;
        grid_map::Position center(0.0, 0.0f);
        double radius = 1.5;


        MDT::Pose defaultPose = {0, 0, 0.5f, 0, 0, 0};
        for (grid_map::CircleIterator iterator(mapData, center, radius); !iterator.isPastEnd(); ++iterator)
        {
            mapData.getPosition(*iterator, tmpP);
            MDT::POINT pointW;

            // // 机器人坐标系下的坐标
            pointW.x() = tmpP.x();
            pointW.y() = tmpP.y();
            pointW.z() = -0.5f;

            if (isInLegsWorkspace(leg_num, defaultPose, pointW) == true)
            {
                Cloud_p.push_back(pointW); // 把单腿的可落足点添加到vector容器中
            }
        }

        return Cloud_p;
    }


    // 机体base坐标系下六个默认落足点
    MDT::POINT init_defaultFoothold(const int &num)
    {
        MatrixXX initFeetPosition_B_Frame(6, 3);
        // initFeetPosition_B_Frame.row(0) << 0.35350, -0.27608, -0.13143; // RF
        // initFeetPosition_B_Frame.row(1) << 0.05350, -0.33608, -0.13143; // RM
        // initFeetPosition_B_Frame.row(2) << -0.35350, -0.27608, -0.13143;// RB
        // initFeetPosition_B_Frame.row(3) << 0.35350,  0.27608, -0.13143; // LF
        // initFeetPosition_B_Frame.row(4) << 0.05350,  0.33608, -0.13143; // LM
        // initFeetPosition_B_Frame.row(5) << -0.35350,  0.27608, -0.13143;// LB
        initFeetPosition_B_Frame.row(0) << 0.35, -0.23-0.05, -0.14-0.1; // RF
        initFeetPosition_B_Frame.row(1) << 0.05, -0.29-0.05, -0.14-0.1; // RM
        initFeetPosition_B_Frame.row(2) << -0.35, -0.23-0.05, -0.14-0.1;// RB
        initFeetPosition_B_Frame.row(3) << 0.35,  0.23+0.05, -0.14-0.1; // LF
        initFeetPosition_B_Frame.row(4) << 0.05,  0.29+0.05, -0.14-0.1; // LM
        initFeetPosition_B_Frame.row(5) << -0.35,  0.23+0.05, -0.14-0.1;// LB
        MDT::POINT p;
        p.x() = initFeetPosition_B_Frame(num, 0);
        p.y() = initFeetPosition_B_Frame(num, 1);
        p.z() = initFeetPosition_B_Frame(num, 2);
        return p;
    }

    // 初始化机器人状态,并赋初值
    MDT::RobotState initRobotState(const MDT::Pose &robotPoseW,  MDT::Vector6b gaitToNow, float moveDirection)
    {
        MDT::RobotState state_;
        state_.initialize();
        state_.pose = robotPoseW;
        state_.faultStateToNow << MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG,MDT::NORMAL_LEG_FLAG;
        for(int i=0;i<6;i++)
        {
            // PLANNING::POINT pnt = {HexapodParameter::transList[i].x, HexapodParameter::transList[i].y, HexapodParameter::transList[i].z};
            state_.feetPosition[i] = MDT::pointRotationAndTrans(HexapodParameter::norminalFoothold_B[i], robotPoseW.getT_W_B());
            state_.feetNormalVector[i] << 0, 0, 1; // 默认法向量竖直向上
            state_.gaitToNow[i] = gaitToNow[i];
            state_.maxNormalForce[i] = 1000.0f;
            state_.frcitionMu[i] = 0.8f;
        }
        state_.moveDirection = moveDirection;

        return state_;
    }



    bool support_leg_inverse_kin(const MDT::Pose &Wolrd_BasePose, const MDT::FeetPositions &World_feet, 
        const std::vector<int> &support_leg, MatrixX3 &joints_output)
    {
        joints_output.resize(support_leg.size(), 3);

        // 计算足端目标机器人坐标系到世界坐标系的旋转平移矩阵T
        Eigen::Isometry3d T_W_B = Wolrd_BasePose.getT_W_B(); //getTrans_W_B(Wolrd_BasePose);

        int row_index = 0;
        // 一个腿一个腿求解
        for (int i = 0; i < 6; ++i)
        {
            auto iter = std::find(support_leg.begin(), support_leg.end(), i + 1);

            if (iter != support_leg.end())
            {
                Eigen::Vector3d p1_ = Eigen::Vector3d(World_feet.footP[i].x(), World_feet.footP[i].y(), World_feet.footP[i].z());
                Eigen::Vector3d foot_FixJi;
                // foot_FixJi = TransMatrix_FixJi_Body[i] * T_W_B.inverse() * p1_; // fixGu坐标系下描述足端位置 //////////////////////////////////////////////

                float tmp_theta1 = atan2(foot_FixJi.y(), foot_FixJi.x());
                float N = foot_FixJi.z();
                float M = 0.0f;
                if (fabs(foot_FixJi.y()) < 0.00000000001f)
                {
                    M = foot_FixJi.x() - 0.18f;
                }
                else
                {
                    M = foot_FixJi.y() / sin(tmp_theta1) - 0.18f;
                }
                if (sqrt(M * M + N * N) > 0.5 + 0.5)
                {
                    return false;
                }
                float tmp_acos = acos((M * M + N * N) / sqrt(M * M + N * N));
                float tmp_theta2 = atan2(N, M) + tmp_acos;
                float tmp_theta3 = atan2(N - 0.5 * sin(tmp_theta2), M - 0.5f * cos(tmp_theta2)) - tmp_theta2;

                joints_output(row_index, 0) = tmp_theta1;
                joints_output(row_index, 1) = tmp_theta2;
                joints_output(row_index, 2) = tmp_theta3 + _PI_ / 2;
                ++row_index;
            }
        }

        return true;
    }

}