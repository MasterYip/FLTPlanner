#include <planning.h>
#include "constrains/dynamic_constrain.hh"
#include "constrains/util.hh"                         //数据类型
#include "constrains/Bretl.hh" //CWC
#include "constrains/my_cdd.hh"
#include "constrains/kinematics_constrain.hh" //运动学约束
#include "constrains/dynamic_constrain.hh"    //动力学约束
#include "HexMini_IK_interface.hpp"

namespace PLANNING
{

    /**
     * @brief 计算reduced kinematic margin  ---- (非通用, 已修改)
     * @par   pnts: 多边形的顶点
     * @par  footW: 足端坐标
     * @par  forwardDirection_W: 足端坐标系下的前进方向, 参考坐标系为World!!!
     * @par  LegNum: 腿号
    */
    float getReduceKinematicMargin(const MDT::Pose &robotPoseW, const MDT::POINT &footW, 
        const float &forwardDirection_W, const int &legNum_)
    // (const int &legNum_, const hit_spider::hexapod_Base_Pose &robotPose, const geometry_msgs::Point &footW, const geometry_msgs::Point &direction_)
    {
        float inverseDirection = forwardDirection_W + _PI_;
        MDT::POINT n_= {cos(inverseDirection), sin(inverseDirection), 0}; // 机体Trunk坐标系下的方向
        // Eigen::Vector3d n_ = Eigen::Vector3d(-direction_.x, -direction_.y, -direction_.z); 

        // 计算足端目标机器人坐标系到世界坐标系的旋转平移矩阵T
        // Eigen::Isometry3d T_W_B = robotPoseW.getT_W_B();

        // // 计算足端到固定基节（扇形坐标系）坐标系下的坐标
        // Eigen::Vector3d p1_ = footW;
        // Eigen::Vector3d foot_FixGu;
        // Eigen::Vector3d foot_Body;
        // foot_Body = T_W_B.inverse() * p1_;
        // foot_FixGu = HexapodParameter::TransMatrix_FixGu_Body[legNum_] * foot_Body;

        // MatrixXX A_output[6];
        // VectorX b_output[6];
        auto Ab = Robot_State_Transition::get_oneLegKinematics_con_cog_foot(robotPoseW, legNum_);

        if(!Robot_State_Transition::isInConvex(Ab.first, Ab.second, footW))
        {
            return -1; // 质心位置不在运动学凸包内部
        }

        auto intersection_ = Robot_State_Transition::findIntersection(Ab.first, Ab.second, footW, n_);

        float reducedMargin =   (intersection_ - footW).norm();

        // intersection_(0) - hexapodState.pose.x;        

        // // 计算机体坐标系下的前进向量
        // Eigen::Vector3d tmp = n_ + foot_Body;
        // // 计算固定股节坐标系下的方向向量
        // // tmp = HexapodParameter::TransMatrix_FixGu_Body[legNum_] * tmp;
        // Eigen::Vector3d nn_ = tmp - foot_FixGu;

        // 计算与扇形的交点
        // MDT::POINT intersectionP = GEOMETRY::getIntersectionSector(USER::LegWorkspaceR, foot_FixGu, nn_);
        // intersectionP.z() = foot_FixGu.z(); // z坐标就是足端的z坐标
        // float reducedMargin = sqrt((intersectionP.x() - foot_FixGu.x()) * (intersectionP.x() - foot_FixGu.x()) +
        //                            (intersectionP.y() - foot_FixGu.y()) * (intersectionP.y() - foot_FixGu.y()));

        return reducedMargin; // 单腿足端能移动的距离
    }




    /**
     * \brief 获得COG到前进方向与支撑多边形的交点的距离 ---- 足式通用
     * \par RobotCenter: 足端坐标
     * \par forwardDirection_W: 足端坐标系下的前进方向, 参考坐标系为World!!!
     * \par pnts: 支撑多边形的顶点
     * 
     * 
    */
    float getMaxLengthCOGtoPolygon(const MDT::POINT &trunkPosition_W, const float &forwardDirection_W ,const std::vector<MDT::POINT> &supportPnts_W)
    {
        MDT::POINT dir_W = {cos(forwardDirection_W), sin(forwardDirection_W), 0};
        MDT::POINT intersection = GEOMETRY::getIntersection(supportPnts_W, trunkPosition_W, dir_W);
        float maxL = sqrt((intersection.x() - trunkPosition_W.x())*(intersection.x() - trunkPosition_W.x()) + (intersection.y() - trunkPosition_W.y()) * (intersection.y() - trunkPosition_W.y())); //  + (intersection.z - footG.z) * (intersection.z - footG.z)

        return maxL;
    }



    //计算静态稳定裕度  ---- 足式通用
    bool getStabilityMargin(MDT::POINT RobotCenter,std::vector<MDT::POINT> Pnt,int n , float *margin_)
    {
        std::vector<float> dis(n);

        if (!GEOMETRY::InPolygon(RobotCenter,n,Pnt)) {return false;}
        for(int i=0;i<n-1;i++)
        {
            dis[i] = GEOMETRY::distancePtSeg(RobotCenter,Pnt[i],Pnt[i+1]);
        }
        dis[n-1] = GEOMETRY::distancePtSeg(RobotCenter,Pnt[n-1],Pnt[0]);

        std::vector<float>::iterator myMin = min_element(dis.begin(), dis.end());

        *margin_ = *myMin;
        return true;
    }






    /**
     * \brief 根据margin和废腿信息进行选取下一帧可行的支撑状态  ---- (六足通用)
     * \param hexapodState 机器人当前状态
     * \return 返回可行的支撑状态集
    */
    std::vector<MDT::Vector6b> findAvailableSupportStates(const MDT::RobotState &hexapodState)
    {
        // 计算fault leg 数目
        int faultLegNum = 0;
        for(int i=0;i<6;i++)
        {
            if (hexapodState.faultStateToNow[i] == hexapodState.faultFlag())
                faultLegNum++;
        }

        //计算允许的support leg状态。
        std::vector<MDT::Vector6b> tmpSupportList;

        // 如果当前状态下有fault leg存在, 将fault leg作为support leg的状态删掉
        if(faultLegNum > 0)
        {
            for(int i=0; i<HexapodParameter::initialSupportList.size();i++)
            {
                int tmpNum_ = 0;
                for(int j=0;j<6;j++)
                {
                    if(hexapodState.faultStateToNow[j] == hexapodState.faultFlag() 
                        && HexapodParameter::initialSupportList[i][j] == hexapodState.supportFlag())
                    {
                        tmpNum_++;
                    }
                }
                // 无使用fault leg作为support leg的情况
                if(tmpNum_ == 0)
                {
                    tmpSupportList.push_back(HexapodParameter::initialSupportList[i]);
                }
            }
        }
        else
        {
            tmpSupportList = HexapodParameter::initialSupportList;
        }

        //去除与上一周期相同的摆动状态,限制机器人不能重复摆动相同的足
        std::vector<MDT::Vector6b> tmpSupportList2;
        for(int i=0;i<tmpSupportList.size();i++)
        {
            int tmpNum_ = 0;
            for(int j=0;j<6;j++)
            {
                if(tmpSupportList[i](j) == MDT::SWING_FLAG)
                {
                    if(hexapodState.gaitToNow[j] == MDT::SWING_FLAG && hexapodState.faultStateToNow[j] != hexapodState.faultFlag())
                    {
                        tmpNum_++;
                    }
                }
            }
            if(tmpNum_ == 0)
            {
                tmpSupportList2.push_back(tmpSupportList[i]);
            }
        }



        std::vector<MDT::Vector6b> availableSupportList;


        //根据静态稳定裕度排除不稳定的状态
        for (int i = 0; i < tmpSupportList2.size();i++)
        {

            //支持多边形顶点
            std::vector<MDT::POINT> Pnts_;
            // std::vector<float> supportLegsReducedMargins;
            for (int j = 0; j < 3; j++)
            {
                if(tmpSupportList2[i](j) == hexapodState.supportFlag())
                {
                    //记录支持腿的坐标，以计算静态稳定裕度
                    Pnts_.push_back(hexapodState.feetPosition[j]);
                }
                /* code */
            }
            for (int j = 5; j >2; j--)
            {
                if(tmpSupportList2[i](j) == hexapodState.supportFlag())
                {
                    //记录支持腿的坐标，以计算静态稳定裕度
                    Pnts_.push_back(hexapodState.feetPosition[j]);
                }
                /* code */
            }

            // 计算静态稳定裕度
            float stabilityMargin_ = 0;
            
            // getStabilityMargin(MDT::POINT RobotCenter,std::vector<MDT::POINT> Pnt,int n , float *margin_)
            bool isInPolygon =  PLANNING::getStabilityMargin(MDT::POINT(hexapodState.pose.x, hexapodState.pose.y, hexapodState.pose.z), 
                Pnts_, Pnts_.size(), &stabilityMargin_);

            if(isInPolygon == true && stabilityMargin_ > 0.1f)
            {
                // cout << stabilityMargin_ << endl;
                availableSupportList.push_back(tmpSupportList2[i]);
            }
        }

        return availableSupportList;
    }

    /** 
     *    **************(通用性取决于调用的getReduceKinematicMargin函数)
     * \brief 根据选取的supportState和reduced kinamatic margin计算可行的步长
     * \param hexapodState 机器人当前状态
     * \param availableSupportList 可行的支撑状态集
     * \return 返回可行的步长集
    */
    std::vector<float> findAvailableStepLength(const MDT::RobotState &hexapodState,
        const std::vector<MDT::Vector6b> &availableSupportList)
    {
        // 计算每条腿的reduced margin *************************************************************************************88
        float reduxceKinematicMargin_[6];
        for(int i=0;i<6;i++)
        {
            reduxceKinematicMargin_[i] = PLANNING::getReduceKinematicMargin(hexapodState.pose, 
                hexapodState.feetPosition[i], hexapodState.moveDirection, i);
        }

        std::vector<float> maxStepLengthForSupportState;
        for (int i = 0; i < availableSupportList.size(); i++)
        {
            //支撑多边形顶点
            std::vector<MDT::POINT> Pnts_;
            std::vector<float> supportLegsReducedMargins;

            for (int j = 0; j < 3; j++)
            {
                if(availableSupportList[i](j) == hexapodState.supportFlag())
                {
                    //记录支持腿的坐标，以计算COG到前进方向与支撑多边形的交点的距离
                    Pnts_.push_back(hexapodState.feetPosition[j]);
                    //记录支撑腿相应的运动学裕度
                    supportLegsReducedMargins.push_back(reduxceKinematicMargin_[j]);
                }
                /* code */
            }
            for (int j = 5; j >= 3; j--)
            {
                if(availableSupportList[i](j) == hexapodState.supportFlag())
                {
                    //记录支持腿的坐标，以计算COG到前进方向与支撑多边形的交点的距离
                    Pnts_.push_back(hexapodState.feetPosition[j]);
                    //记录支撑腿相应的运动学裕度
                    supportLegsReducedMargins.push_back(reduxceKinematicMargin_[j]);
                }
                /* code */
            }


            // 获得COG到前进方向与支撑多边形的交点的距离
            MDT::POINT trunkP_ = {hexapodState.pose.x,hexapodState.pose.y, hexapodState.pose.z};
            float COGtoPolygon_ = PLANNING::getMaxLengthCOGtoPolygon(trunkP_, hexapodState.moveDirection, Pnts_);

            // float COGtoPolygon_ = ParameterHexapod->getMaxLengthCOGtoPolygon
            //     (hexapodState.poseNow.position,hexapodState.moveDirectionNext, Pnts_, Pnts_.size());

            supportLegsReducedMargins.push_back(COGtoPolygon_);
            
            int minIndex = min_element(supportLegsReducedMargins.begin(),supportLegsReducedMargins.end()) - supportLegsReducedMargins.begin();

            maxStepLengthForSupportState.push_back(supportLegsReducedMargins[minIndex]);
        }

        return maxStepLengthForSupportState;
        
    }


    /**
     * \brief 根据关节力矩约束计算可行的步长
     * \param hexapodState 机器人当前状态
     * \param supportLegNumList 支撑腿的序列集
     * \param contactPoints 支撑腿的落足点位置
     * \param forwardDir 前进方向向量
     * \param stepLength 需检查的步长
    */
    float getMaximumTransitionLength_considerJntTorqueConstraint(const MDT::RobotState& hexapodState, const std::vector<int>& supportLegNumList, 
        const MatrixX3 &contactPoints, const Eigen::Vector3f& forwardDir, float stepLength)
    {
        Eigen::Vector3f direction = forwardDir;
        Eigen::Vector3f start = {hexapodState.pose.x, hexapodState.pose.y, hexapodState.pose.z};
        direction.normalize(); // 归一化!!
        // 计算步长
        float step = stepLength / 9.0;
        float roll_delta = 0.0/9.0;  // 先忽略roll,pitch,yaw的变化
        float pitch_delta = 0.0/9.0;
        float yaw_delta = 0.0/9.0;
        std::vector<MDT::Pose> dicresized_pose;
        // 生成十个中间点
        for (int i = 0; i <= 9; ++i) {
            Eigen::Vector3f point = start + i * step * direction;
            MDT::Pose pose = {point(0), point(1), point(2), hexapodState.pose.roll+i*roll_delta, hexapodState.pose.pitch+i*pitch_delta, hexapodState.pose.yaw+i*yaw_delta};
            dicresized_pose.push_back(pose);
            // std::cout << (point-start).norm() << std::endl;
        }

        MatrixXX feetPosition_W(6, 3);
        for (int i = 0; i < 6; ++i)
        {
            feetPosition_W.row(i) << hexapodState.feetPosition[i].x(), hexapodState.feetPosition[i].y(), hexapodState.feetPosition[i].z();
        }

        int support_leg_num = supportLegNumList.size();
        MatrixX3 joints(support_leg_num, 3);
        joints.setZero();  

        int sucessCount = 0;
        for (auto pose_ : dicresized_pose) {
            sucessCount++;
            auto result = wholeBodyIK(pose_.x, pose_.y, pose_.z, pose_.roll, pose_.pitch, pose_.yaw, feetPosition_W);  // need to modify as single leg IK to accelerate it.
            if(result.first == false)
            {
                break;
            }

            Eigen::Matrix<double, Eigen::Dynamic, 1> jntTheta = result.second;

            for(int jjj=0; jjj<support_leg_num; jjj++)
            {
                joints.row(jjj) << jntTheta((supportLegNumList[jjj]-1)*3 + 0), jntTheta((supportLegNumList[jjj]-1)*3 + 1), jntTheta((supportLegNumList[jjj]-1)*3 + 2);
            }
            // std::cout <<joints << std::endl;

            if(!Robot_State_Transition::is_meet_dynamic_con2(Vector3(pose_.x, pose_.y, pose_.z), supportLegNumList, contactPoints, joints, USER::HexMini_PinoModel))
            {
                break;
            }

            // std::cout << "point: " << pnt_.transpose() << std::endl;
        }
        
        return (sucessCount-1)*step;
    }


    bool supportLegCollisionCheck(const MDT::RobotState& hexapodState, const std::vector<int>& supportLegNumList, 
            const Eigen::Vector3f& forwardDir, float stepLength, const grid_map::GridMap& mapData)
    {
        Eigen::Vector3f direction = forwardDir;
        Eigen::Vector3f start = {hexapodState.pose.x, hexapodState.pose.y, hexapodState.pose.z};
        direction.normalize(); // 归一化!!
        // 计算步长
        float step = stepLength / 9.0;
        float roll_delta = 0.0/9.0;  // 先忽略roll,pitch,yaw的变化
        float pitch_delta = 0.0/9.0;
        float yaw_delta = 0.0/9.0;
        std::vector<MDT::Pose> dicresized_pose;
        // 生成十个中间点
        for (int i = 0; i <= 9; ++i) {
            Eigen::Vector3f point = start + i * step * direction;
            MDT::Pose pose = {point(0), point(1), point(2), hexapodState.pose.roll+i*roll_delta, hexapodState.pose.pitch+i*pitch_delta, hexapodState.pose.yaw+i*yaw_delta};
            dicresized_pose.push_back(pose);
            // std::cout << (point-start).norm() << std::endl;
        }

        MatrixXX feetPosition_W(6, 3);
        for (int i = 0; i < 6; ++i)
        {
            feetPosition_W.row(i) << hexapodState.feetPosition[i].x(), hexapodState.feetPosition[i].y(), hexapodState.feetPosition[i].z();
            // std::cout << feetPosition_W.row(i) << std::endl;
        }
        

        int support_leg_num = supportLegNumList.size();
        MatrixX3 joints(support_leg_num, 3);
        joints.setZero();  

        int sucessCount = 0;
        for (auto pose_ : dicresized_pose) {
            sucessCount++;

            // 逆运动学求解
            auto result = wholeBodyIK(pose_.x, pose_.y, pose_.z, pose_.roll, pose_.pitch, pose_.yaw, feetPosition_W);  // need to modify as single leg IK to accelerate it.
            if(result.first == false)
            {
                return false;
            }
            Eigen::VectorXd q_not_in_order = result.second;

            // std::cout << q.transpose() << std::endl;
            // RF RM RB LF LM LB
            //   0     1    2    3   4   5
            // 12-15 15-18 9-12 3-6 6-9 0-3 

            for(int i = 0; i< supportLegNumList.size(); i++)
            {
                // 更改为pinocchio的顺序接口
                Eigen::VectorXd q = Eigen::VectorXd::Zero(18);
                q.segment(12, 3) = q_not_in_order.segment(0, 3);
                q.segment(15, 3) = q_not_in_order.segment(3, 3);
                q.segment(9, 3) = q_not_in_order.segment(6, 3);
                q.segment(3, 3) = q_not_in_order.segment(9, 3);
                q.segment(6, 3) = q_not_in_order.segment(12, 3);
                q.segment(0, 3) = q_not_in_order.segment(15, 3);

                // pinocchio正运动学，生成碰撞检测点
                pinocchio::forwardKinematics(USER::HexMini_PinoModel, USER::HexMini_PinoModel_Data, q);
                pinocchio::updateFramePlacements(USER::HexMini_PinoModel, USER::HexMini_PinoModel_Data);
                auto feetP_Body = USER::HexMini_PinoModel_Data.oMf[USER::HexMini_PinoModel.getFrameId(USER::FootName[supportLegNumList[i]-1])].translation();
                auto shankP_Body = USER::HexMini_PinoModel_Data.oMf[USER::HexMini_PinoModel.getFrameId(USER::ShankName[supportLegNumList[i]-1])].translation();
                auto midP_Body = (shankP_Body + feetP_Body) / 2;
                auto thighP_Body = USER::HexMini_PinoModel_Data.oMf[USER::HexMini_PinoModel.getFrameId(USER::ThighName[supportLegNumList[i]-1])].translation();

                Eigen::Isometry3d T_W_B = pose_.getT_W_B();

                // auto feetP_World = T_W_B * feetP_Body;
                auto shankP_World = T_W_B * shankP_Body;
                auto midP_World = T_W_B * midP_Body;
                auto thighP_World = T_W_B * thighP_Body;
                std::vector<Eigen::Vector3d> keyPnts;
                // keyPnts.push_back(feetP_World);
                keyPnts.push_back(shankP_World);
                keyPnts.push_back(midP_World);
                keyPnts.push_back(thighP_World);


                for(int i = 0; i< keyPnts.size(); i++)
                {
                    if(!COLLISION_CHECK::isCollision_onePoint(keyPnts[i], mapData))
                    {
                        std::cout << "collision" << std::endl;
                        return false;
                    }
                }

            }

        }

        return true;
    }

    // 根据落足点拟合的平面，计算沿着平面的三维方向向量
    std::pair<Vector3,Vector3>  calculateForwardVector3d(const MDT::RobotState& hexapodState)
    {
        // 通过最小二乘拟合所有接触点的平面, 并计算该平面与xy平面的夹角， 用于计算机器人的roll和pitch角度
        std::vector<Vector3> pnts;
        float sumZ = 0.0f;
        int ccount = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(hexapodState.faultStateToNow[j] != MDT::FAULT_LEG_FLAG)
            {
                pnts.push_back(Vector3(hexapodState.feetPosition[j]));
                sumZ += hexapodState.feetPosition[j].z();
                ccount++;
            }
        }
        // pnts.push_back(Vector3(hexapodState.pose.x+0.4, hexapodState.pose.y, hexapodState.pose.z +  1*(sumZ/(float)ccount) + USER::norminalTrunkHeight - hexapodState.pose.z) );

        Vector3 n = Vector3(cos(hexapodState.moveDirection),sin(hexapodState.moveDirection),0); // 假设向量n在参考坐标系下的值为(0, 0, 1)
        // 计算平面法向量
        Vector3 planeNormal = Robot_State_Transition::calculatePlaneNormal(pnts);
        // 计算指定向量n在平面上的投影
        Vector3 projection = n - n.dot(planeNormal) / planeNormal.squaredNorm() * planeNormal;  // 这就是我要的前进方向的三维向量

        return std::make_pair(planeNormal, projection);
    }


    // 稳定性约束, 运动学约束, 力矩约束, (膨胀约束, 足地力约束)
    std::pair<std::vector<MDT::Vector6b>, std::vector<MDT::POINT>> getSupportListAndStepL_underConstrains(const MDT::RobotState hexapodState, const grid_map::GridMap &mapData)
    {


        std::vector<MDT::RobotState> resultLists;
        std::vector<MDT::POINT> maxMoving;

        std::vector<MDT::Vector6b> availableSupportState_ = PLANNING::findAvailableSupportStates(hexapodState);
        if(availableSupportState_.size() == 0)
        {
            return std::make_pair(availableSupportState_, maxMoving);
        }

        assert(availableSupportState_.size() != 0 && "availableSupportState_ should not be empty22.");
        
        // 根据落足点拟合的平面，计算沿着平面的三维方向向量 和 平面法向量
        auto results_fitPlane = PLANNING::calculateForwardVector3d(hexapodState);
        Vector3 planeNormal = results_fitPlane.first;
        Vector3 projection = results_fitPlane.second;


        //#########计算机体旋转量，保持与平面平行#####################################################################################################
        Vector3 x_n = Vector3(1,0,0); 
        Vector3 y_n = Vector3(0,1,0); 

        // 计算指定向量n在平面上的投影
        Vector3 projection_xn = x_n - x_n.dot(planeNormal) / planeNormal.squaredNorm() * planeNormal; 

        Vector3 projection_yn = y_n - y_n.dot(planeNormal) / planeNormal.squaredNorm() * planeNormal;
        // 计算pitch角度
        double pitch = -atan2(projection_xn(2), projection_xn(0));
        
        // 计算roll角度
        double roll = atan2(projection_yn(2), projection_yn(1));


        //#########开始筛选获得可行步态#####################################################################################################
        std::vector<MDT::Vector6b> availableSupportState_Final;
        for (int i = 0; i < availableSupportState_.size(); ++i)
        {
            // 数据准备************************************************************************
            int support_leg_num = 0; // 支撑腿数量
            for (int j = 0; j < 6; ++j)
            {
                if(availableSupportState_[i][j] == MDT::SUPPORT_FLAG)
                    support_leg_num += 1;
            }

            std::vector<int> support_leg;                 // 支撑腿标号
            MatrixX3 contact_Points(support_leg_num, 3);  // 支撑腿接触点
            MatrixXX CWC_inputs(support_leg_num, 2);      // 支撑点的x,y坐标
            MatrixX3 contact_normals(support_leg_num, 3); // 接触点法向量
            contact_normals.setZero();
            contact_normals.col(2).setOnes(); // 先考虑简单情况, 将第三列设置为1, 其余元素不变. 即接触点法向量为(0,0,1)

            VectorX contact_maxmumF(support_leg_num); // 接触点最大地面反作用力
            
            int row_index = 0;

            // 记录接触点数据
            for (int j = 0; j < 6; ++j)
            {
                if (availableSupportState_[i][j] == MDT::SUPPORT_FLAG)  // 支撑
                {
                    // contact_maxmumF(row_index)  = 1000;
                    contact_maxmumF(row_index)  = hexapodState.maxNormalForce[j];


                    CWC_inputs.row(row_index) << hexapodState.feetPosition[j].x(),
                        hexapodState.feetPosition[j].y();
                    
                    contact_Points.row(row_index++) << hexapodState.feetPosition[j].x(),
                        hexapodState.feetPosition[j].y(),
                        hexapodState.feetPosition[j].z();


                    support_leg.push_back(j + 1);  // 记录几号腿为支撑腿
                }
            }





            // MatrixXX A;
            // VectorX b;
            MatrixXX D;
            VectorX d;
            VectorX g, minBound, maxBound;

            // 1.判断是否在力学稳定支撑多边形内部*************************************************************************
            double dis_cwc = 0.0;
            MatrixXX CWC_Vertices;

            if(USER::IsMaxForceConstraint)
            {
                if(!Robot_State_Transition::comput_friction_region_considerMaxF(contact_Points, contact_normals, contact_maxmumF, 0.2, Robot_State_Transition::Mass, CWC_Vertices))
                {
                    continue;
                }
            }
            else
            {
                if (!Robot_State_Transition::comput_friction_region(contact_Points, contact_normals, 0.2, Robot_State_Transition::Mass, CWC_Vertices))
                {
                    continue;
                }
            }


            if(CWC_Vertices.rows() < 3)
            {
                continue;
            }


            // 将平面凸多边形顶点转换为不等式形式
            auto Ab_cwc = Robot_State_Transition::Bretl_Vettices_to_face(CWC_Vertices);

            // 判断质心位置是否在cwc内部, 其实用判断是否在凸多边形内就可以了.
            VectorX is_in_cwc = Ab_cwc.first * point_Planar(hexapodState.pose.x, hexapodState.pose.y);

            if(!((is_in_cwc.array() <= Ab_cwc.second.array()).all()))
            {
                continue; // 质心位置不在cwc内部
            }

            // 求前进方向到cwc的交点
            MatrixXX A_cwc(Ab_cwc.first.rows(), 3);
            A_cwc.block(0, 0, Ab_cwc.first.rows(), 2) = Ab_cwc.first;
            A_cwc.col(2).setZero(); // 因为这个稳定region是平面，所以第三列为0
            auto intersectionP_CWC =  Robot_State_Transition::findIntersection(A_cwc, Ab_cwc.second, 
                Vector3(hexapodState.pose.x, hexapodState.pose.y, 0), 
                Vector3(projection(0),projection(1),0)); 
            Vector3 COG_W;
            COG_W << hexapodState.pose.x, hexapodState.pose.y, hexapodState.pose.z;
            dis_cwc = sqrt((COG_W.x() - intersectionP_CWC.x()) * (COG_W.x() - intersectionP_CWC.x()) +
                (COG_W.y() - intersectionP_CWC.y()) * (COG_W.y() - intersectionP_CWC.y()) ); //  + (intersectionP.z - footG.z) * (intersectionP.z - footG.z
            // (COG_W - intersectionP_CWC).norm();   // xy平面，水平移动的最大步长 ？


            
            // 2.当前支撑腿对质心的运动约束，求解最大前进步长*****************************************************************************
            auto feet_con_cog_Ab = Robot_State_Transition::get_kinematics_con_foot_cog(hexapodState.pose, support_leg, contact_Points, contact_normals);


            if(!Robot_State_Transition::isInConvex( feet_con_cog_Ab.first, feet_con_cog_Ab.second, COG_W))
            {
                continue; // 质心位置不在运动学凸包内部
            }

            auto intersection_ = Robot_State_Transition::findIntersection(feet_con_cog_Ab.first, feet_con_cog_Ab.second, COG_W,
                projection);


            double maxDeltaZ = intersection_(2) - hexapodState.pose.z;

            double maxL_ = (COG_W - intersection_).norm();

            double kin_reslut_length = sqrt(maxL_ * maxL_ - maxDeltaZ * maxDeltaZ); // 水平移动的最大步长 ？



            // 3.选择的支撑状态满足关节扭矩约束，接着确定最大步长 ***********************************************************************
            // 应该根据前进方向，离散化之前确定的最大步长， 然后逐点判断是否满足扭矩约束
            // 注意这里的前进方向一定需要和最终的保持一致，否则这里的工作就失去意义了。
            double length_ = std::min(dis_cwc, kin_reslut_length);


            Eigen::Vector3f forwardDir(projection(0), projection(1), projection(2));
            if(USER::IsToruqeLimitConstraint)
            {
                length_ = getMaximumTransitionLength_considerJntTorqueConstraint(hexapodState, support_leg, contact_Points, forwardDir, length_);
            }
            

            // 4.支撑腿的碰撞检测 ***********************************************************************
            // 为了加快速度，collsion check只保留了摆动腿的足端轨迹检查； 这里临时屏蔽掉了
            // if(USER::COLLISION_CHECK)
            // {
            //     if(!supportLegCollisionCheck(hexapodState, support_leg, forwardDir, length_, mapData))
            //     {
            //         continue;
            //     }                    
            // }


            if(length_ < 0.01) continue;
            
            MDT::POINT maxMovingPoint = {length_, pitch, maxDeltaZ};
            maxMoving.push_back(maxMovingPoint);
            availableSupportState_Final.push_back(availableSupportState_[i]);
        }


        return std::make_pair(availableSupportState_Final, maxMoving);
    }

    
    // 获取可行的旋转support状态
    // deltaYaw 是目标角度与当前角度的差值
    std::pair<std::vector<MDT::Vector6b>, std::vector<float>> getSupportList_Rotate_underConstrains(const MDT::RobotState hexapodState, 
        const grid_map::GridMap &mapData)
    {
        std::vector<float> maxRotationAngle;

        // 根据准静态稳定性等条件进行初步筛选
        std::vector<MDT::Vector6b> availableSupportState_ = PLANNING::findAvailableSupportStates(hexapodState);
        
        
        if(availableSupportState_.size() == 0)
        {
            return std::make_pair(availableSupportState_, maxRotationAngle);
        }


        // std::vector<MDT::Vector6b> availableSupportState_Final;



        for (int i = 0; i < availableSupportState_.size(); ++i)
        {
            MDT::RobotState testState = hexapodState;
            testState.gaitToNow = availableSupportState_[i];
            auto yawAngleRange = getMaxYawRotationAngle(testState);

            if(hexapodState.moveDirection - hexapodState.pose.yaw < 0)
            {
                maxRotationAngle.push_back(yawAngleRange.first);
            }
            else
            {
                maxRotationAngle.push_back(yawAngleRange.second);
            }
        }
        
        return std::make_pair(availableSupportState_, maxRotationAngle);
    }

    


    ContactsInfos get_now_Feasible_foot_position(const MDT::Pose &pose, const grid_map::GridMap &mapData)
    {
        // MatrixX3 AA;
        // MatrixX3 NormalVector;
        // VectorX  frcitionMu;
        // VectorX  maxNormalF;

        ContactsInfos contactDates;


        int num = 0;

        grid_map::Position position;
        grid_map::Position center(pose.x, pose.y);
        double radius = 1.5;

        for (grid_map::CircleIterator it(mapData, center, radius); !it.isPastEnd(); ++it)
        {
            if(mapData.isValid(*it,"elevation"))
            {
                ++num;
            }

            // mapData.getPosition(*it, position);
            // if (mapData.at("elevation", *it) == 0)
            // {
            //     ++num;
            // }
        }

        contactDates.position.resize(num, 3);
        contactDates.normalVector.resize(num, 3);
        contactDates.frcitionMu.resize(num);
        contactDates.maxNormalF.resize(num);

        // AA.resize(num, 3);
        // NormalVector.resize(num, 3);
        // frcitionMu.resize(num);
        // maxNormalF.resize(num);


        int row_index = 0;

        for (grid_map::CircleIterator it(mapData, center, radius); !it.isPastEnd(); ++it)
        {
            // grid_map::Position position;
            // mapData.getPosition(*it, position);
            if(mapData.isValid(*it,USER::elevationLayerName))
            {
                grid_map::Position tmpP_;
                mapData.getPosition(*it, tmpP_);
                contactDates.normalVector.row(row_index) << mapData.at(USER::normalVector_x_Name, *it), mapData.at(USER::normalVector_y_Name, *it), mapData.at(USER::normalVector_z_Name, *it);
                contactDates.position.row(row_index) << tmpP_.x(), tmpP_.y(), mapData.at(USER::elevationLayerName, *it);
                // contactDates.frcitionMu(row_index) = mapData.at("friction_mu", *it);
                // contactDates.maxNormalF(row_index) = mapData.at("max_normal_force", *it);
                contactDates.maxNormalF(row_index) =  1000 ; //- 100*tmpP_.x();
                row_index++;

            }
        }

        return contactDates;
    }

    MDT::AvailableContactsInfo getAvailableFootholds(const MDT::RobotState &hexapodState, const grid_map::GridMap &mapData)
    {
        MDT::AvailableContactsInfo availableContacts;

        // 环境可落足点集合 //这里是获得机体周围一个圆形区域的点
        auto result = get_now_Feasible_foot_position(hexapodState.pose, mapData);

        MatrixX3 map_feasible_position = result.position;

        MatrixXX A_output[6];
        VectorX b_output[6];
        // base约束foot
        Robot_State_Transition::get_kinematics_con_cog_foot(hexapodState.pose, A_output, b_output);

        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.gaitToNow[i] == MDT::SWING_FLAG)
            {
                MatrixX3 A = A_output[i];
                VectorX b = b_output[i];

                // 判断环境中的点是否在kin质心约束foot的约束范围内
                MatrixXX temp = A * map_feasible_position.transpose();
                for (int j = 0; j < temp.cols(); ++j)
                {
                    if ((temp.col(j).array() <= b.array()).all())
                    {
                        availableContacts.position.leg[i].push_back(map_feasible_position.row(j).transpose());
                        availableContacts.normalVector.leg[i].push_back(result.normalVector.row(j).transpose());
                        availableContacts.frcitionMu[i].push_back(result.frcitionMu(j));
                        availableContacts.maxNormalF[i].push_back(result.maxNormalF(j));
                    }
                }
            }
        }
        return availableContacts;
    }




    bool swingLegFoot_position_Expert_forSim2(MDT::RobotState &hexapodState, const grid_map::GridMap &mapData)
    {
        // MDT::VectorList feasible_positions;
        for (int i = 0; i < 6; ++i)
        {
            if (hexapodState.gaitToNow[i] == MDT::SWING_FLAG)
            {
                Eigen::Vector3d nF = HexapodParameter::norminalFoothold_B[i];
                Vector3 default_p0 = hexapodState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0, nF.y(), nF.z());
                Vector3 default_p1 = hexapodState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.1, nF.y(), nF.z());
                Vector3 default_p2 = hexapodState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.2, nF.y(), nF.z());

                // 检查default_p是否在地图中
                std::vector<Vector3> feasible_positions;

                grid_map::Position tmpP_;
                tmpP_ << default_p0.x(), default_p0.y();
                grid_map::Index it11;
                mapData.getIndex(tmpP_,it11);
                if(mapData.isValid(it11,"elevation"))
                {
                    default_p0.z() = mapData.at("elevation",it11);
                    feasible_positions.push_back(default_p0);
                }

                tmpP_ << default_p1.x(), default_p1.y();
                grid_map::Index it12;
                mapData.getIndex(tmpP_,it12);
                if(mapData.isValid(it12,"elevation"))
                {
                    default_p1.z() = mapData.at("elevation",it12);
                    feasible_positions.push_back(default_p1);
                }

                tmpP_ << default_p2.x(), default_p2.y();
                grid_map::Index it13;
                mapData.getIndex(tmpP_,it13);
                if(mapData.isValid(it13,"elevation"))
                {
                    default_p2.z() = mapData.at("elevation",it13);
                    feasible_positions.push_back(default_p2);
                }


                if (feasible_positions.size() != 0)
                {
                    // 获得一个0到10的随机数
                    int maxRow = rand() % feasible_positions.size();

                    hexapodState.feetPosition[i].x() = feasible_positions[maxRow](0);
                    hexapodState.feetPosition[i].y() = feasible_positions[maxRow](1);
                    hexapodState.feetPosition[i].z() = feasible_positions[maxRow](2);
                    hexapodState.feetNormalVector[i].x() = 0;
                    hexapodState.feetNormalVector[i].y() = 0;
                    hexapodState.feetNormalVector[i].z() = 1;

                    hexapodState.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                }
                else
                {
                    // return false;
                    hexapodState.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;

                    Eigen::Vector3d dafult_position = HexapodParameter::norminalFoothold_B[i];
                    dafult_position(2) += 0.3;
                    dafult_position = hexapodState.pose.getT_W_B() * dafult_position;

                    hexapodState.feetPosition[i].x() = dafult_position.x();
                    hexapodState.feetPosition[i].y() = dafult_position.y();
                    hexapodState.feetPosition[i].z() = dafult_position.z();
                }
            }
        }
        return true;
    }




/**
     * \brief 根据当前状态 计算下一步可行接触状态 (挑选最大步长步态)
     * \param hexapodState 机器人当前状态
     * \param mapData  地图数据
     * \return 返回下一步可行接触状态
    */
    MDT::RobotState getNextMCTSstateByExpert_forSim(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {

        std::vector<MDT::RobotState> resultLists;
        std::vector<MDT::Vector6b> availableSupportState_ = PLANNING::findAvailableSupportStates(rState);

        if(availableSupportState_.size() == 0)
        {
            return rState;
        }
        assert(availableSupportState_.size() > 0 && "availableSupportState_ should not be empty.");

        std::vector<float> maxStepLengthForSupportState = findAvailableStepLength(rState,availableSupportState_);
        
        //获得maxStepLengthForSupportState中最大元素的索引
        int maxIndex = std::max_element(maxStepLengthForSupportState.begin(), maxStepLengthForSupportState.end()) - maxStepLengthForSupportState.begin();
        MDT::RobotState hexapodState = rState;
        hexapodState.gaitToNow = availableSupportState_[maxIndex];
        float stepLength_ = maxStepLengthForSupportState[maxIndex]*2.0/3.0;

        if(stepLength_ > 0.05f)
        {
            // 防止落在支撑多边形边上
            stepLength_ -= 0.05f;
        }
        else if(stepLength_ < 0.01) //防止在仿真过程中机器人不断挪动，由于数值问题移动出扇形工作空间
        {
            stepLength_ = 0;
        }
        

        hexapodState.pose.x += stepLength_ * cos(hexapodState.moveDirection);
        hexapodState.pose.y += stepLength_ * sin(hexapodState.moveDirection);
        // z轴方向的移动
        float averageFootholdsHeight = 0.0f;
        int count = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(rState.faultStateToNow[j] == MDT::NORMAL_LEG_FLAG)
            {
                count++;
                averageFootholdsHeight += rState.feetPosition[j].z();
            }
        }
        averageFootholdsHeight/=float(count);
        hexapodState.pose.z = averageFootholdsHeight + USER::norminalTrunkHeight;

        //findAvailableFootholdsCombination
        //findRandomFootholdsCombination
        if(PLANNING::swingLegFoot_position_Expert_forSim2(hexapodState, mapData)) 
        {
            return hexapodState;
        }

        return rState;



        // //随机选择一个lists里的元素
        // if(lists.size() == 0) return rState;
        // int randomIndex = rand() % lists.size();
        // return lists[randomIndex];
    }



    std::vector<MDT::RobotState> swingLegFoot_position_lists(const MDT::RobotState &rState, const grid_map::GridMap &mapData, float addL)
    {
        auto resultF = getAvailableFootholds(rState, mapData);
        MDT::VectorList feasible_positions = resultF.position;
        MDT::VectorList feasible_normalVector = resultF.normalVector;
        MDT::RobotState hexapodState1 = rState;
        MDT::RobotState hexapodState2 = rState;
        MDT::RobotState hexapodState3 = rState;

        std::vector<MDT::RobotState> resultLists;

        for (int i = 0; i < 6; ++i)
        {
            if (rState.gaitToNow[i] == MDT::SWING_FLAG)
            {
                if (feasible_positions.leg[i].size() != 0)
                {
                    // 寻找x最大的落足点
                    // auto max_iter = std::max_element(feasible_positions.leg[i].begin(), feasible_positions.leg[i].end(),
                    //                                  [](const Vector3 &v1, const Vector3 &v2)
                    //                                  { return v1(0) < v2(0); }); // 找x坐标最大的点

                    // hexapodState1.feetPosition[i].x() = max_iter->x();
                    // hexapodState1.feetPosition[i].y() = max_iter->y();
                    // hexapodState1.feetPosition[i].z() = max_iter->z();
                    // hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).x();
                    // hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).y();
                    // hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).z();

                    // // 获得一个随机落足点
                    // maxRow = rand() % feasible_positions.leg[i].size();
                    // hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    // hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    // hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    // hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    // hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    // hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();

                    // 寻找最靠近默认位置的落足点
                    Eigen::Vector3d nF = HexapodParameter::norminalFoothold_B[i];
                    Vector3 default_p = rState.pose.getT_W_B() * nF;
                    VectorX dis2(feasible_positions.leg[i].size());
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    VectorX::Index maxRow;
                    dis2.minCoeff(&maxRow);
                    hexapodState2.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState2.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState2.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState2.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState2.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState2.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState2.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState2.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    Vector3 default_p1 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x(), nF.y(), nF.z()) + Eigen::Vector3d(addL*cos(rState.pose.yaw),addL*sin(rState.pose.yaw),0);
                    Vector3 default_p2 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x(), nF.y(), nF.z()) + Eigen::Vector3d(addL*cos(rState.pose.yaw),addL*sin(rState.pose.yaw),0);
                    // Vector3 default_p3 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.5, nF.y()-0.5, nF.z());
                    

                    default_p = default_p1;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState1.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState1.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState1.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState1.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState1.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    default_p = default_p2;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    // hexapodState3.feetPosition[i] = default_p2;
                    hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState3.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState3.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    hexapodState1.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                    hexapodState2.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                    hexapodState3.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                }
                else
                {
                    Eigen::Vector3d dafult_position = HexapodParameter::norminalFoothold_B[i];
                    dafult_position(2) += 0.1;
                    dafult_position = hexapodState1.pose.getT_W_B() * dafult_position;
                    hexapodState1.feetPosition[i].x() = dafult_position.x();
                    hexapodState1.feetPosition[i].y() = dafult_position.y();
                    hexapodState1.feetPosition[i].z() = dafult_position.z();

                    hexapodState2.feetPosition[i] = hexapodState1.feetPosition[i];  
                    hexapodState3.feetPosition[i] = hexapodState1.feetPosition[i];

                    hexapodState1.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                    hexapodState2.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                    hexapodState3.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                }
            }
        }

        // resultLists.push_back(hexapodState1);
        // resultLists.push_back(hexapodState2);
        resultLists.push_back(hexapodState3);

        return resultLists;
    }


    MDT::VectorList swingLegFoot_position_visualize(const MDT::RobotState &rState, const grid_map::GridMap &mapData)
    {
        auto resultF = getAvailableFootholds(rState, mapData);
        MDT::VectorList feasible_positions = resultF.position;
        MDT::VectorList feasible_normalVector = resultF.normalVector;
        MDT::RobotState hexapodState1 = rState;
        MDT::RobotState hexapodState2 = rState;
        MDT::RobotState hexapodState3 = rState;

        MDT::VectorList resultsFeetPosition;

        std::vector<MDT::RobotState> resultLists;

        for (int i = 0; i < 6; ++i)
        {
            if (rState.gaitToNow[i] == MDT::SWING_FLAG)
            {
                if (feasible_positions.leg[i].size() != 0)
                {
                    // 寻找x最大的落足点
                    // auto max_iter = std::max_element(feasible_positions.leg[i].begin(), feasible_positions.leg[i].end(),
                    //                                  [](const Vector3 &v1, const Vector3 &v2)
                    //                                  { return v1(0) < v2(0); }); // 找x坐标最大的点

                    // hexapodState1.feetPosition[i].x() = max_iter->x();
                    // hexapodState1.feetPosition[i].y() = max_iter->y();
                    // hexapodState1.feetPosition[i].z() = max_iter->z();
                    // hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).x();
                    // hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).y();
                    // hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(max_iter - feasible_positions.leg[i].begin()).z();

                    // // 获得一个随机落足点
                    // maxRow = rand() % feasible_positions.leg[i].size();
                    // hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    // hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    // hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    // hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    // hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    // hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();

                    // 寻找最靠近默认位置的落足点
                    Eigen::Vector3d nF = HexapodParameter::norminalFoothold_B[i];
                    Vector3 default_p = rState.pose.getT_W_B() * nF;
                    VectorX dis2(feasible_positions.leg[i].size());
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    VectorX::Index maxRow;
                    dis2.minCoeff(&maxRow);
                    hexapodState2.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState2.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState2.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState2.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState2.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState2.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState2.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState2.frcitionMu[i] = resultF.frcitionMu[i][maxRow];
                    resultsFeetPosition.leg[i].push_back(default_p);


                    Vector3 default_p1 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x(), nF.y(), nF.z()) + Eigen::Vector3d(0.1*cos(rState.pose.yaw),0.1*sin(rState.pose.yaw),0);
                    Vector3 default_p2 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x(), nF.y(), nF.z()) + Eigen::Vector3d(0.1*cos(rState.pose.yaw),0.1*sin(rState.pose.yaw),0);
                    // Vector3 default_p3 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.5, nF.y()-0.5, nF.z());
                    

                    default_p = default_p1;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState1.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState1.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState1.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState1.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState1.frcitionMu[i] = resultF.frcitionMu[i][maxRow];
                    // resultsFeetPosition.leg[i].push_back(hexapodState1.feetPosition[i]);

                    default_p = default_p2;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState3.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState3.frcitionMu[i] = resultF.frcitionMu[i][maxRow];
                    // resultsFeetPosition.leg[i].push_back(hexapodState3.feetPosition[i]);

                    hexapodState1.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                    hexapodState2.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                    hexapodState3.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                }
                else
                {
                    Eigen::Vector3d dafult_position = HexapodParameter::norminalFoothold_B[i];
                    dafult_position(2) += 0.3;
                    dafult_position = hexapodState1.pose.getT_W_B() * dafult_position;
                    hexapodState1.feetPosition[i].x() = dafult_position.x();
                    hexapodState1.feetPosition[i].y() = dafult_position.y();
                    hexapodState1.feetPosition[i].z() = dafult_position.z();

                    hexapodState2.feetPosition[i] = hexapodState1.feetPosition[i];  
                    hexapodState3.feetPosition[i] = hexapodState1.feetPosition[i];

                    hexapodState1.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                    hexapodState2.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                    hexapodState3.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                }
            }
        }

        resultLists.push_back(hexapodState1);
        resultLists.push_back(hexapodState2);
        resultLists.push_back(hexapodState3);

        return resultsFeetPosition;
    }


    std::vector<MDT::RobotState> swingLegFoot_position_lists_afterCollisionCheck(MDT::Vector3 &bodyPositionBegin, MDT::Vector3 &bodyPoseBegin, const MDT::RobotState &rState, 
        const grid_map::GridMap &mapData)
    {
        auto resultF = getAvailableFootholds(rState, mapData);
        MDT::VectorList feasible_positions = resultF.position;
        MDT::VectorList feasible_normalVector = resultF.normalVector;
        MDT::RobotState hexapodState1 = rState;
        MDT::RobotState hexapodState2 = rState;
        MDT::RobotState hexapodState3 = rState;

        std::vector<MDT::RobotState> resultLists;
        bool flag1 = true;
        bool flag2 = true;
        bool flag3 = true;

        MDT::Vector3 bodyPositionEnd;
        bodyPositionEnd << rState.pose.x, rState.pose.y, rState.pose.z;

        MDT::Vector3 bodyPoseEnd;
        bodyPoseEnd << rState.pose.roll, rState.pose.pitch, rState.pose.yaw;


        for (int i = 0; i < 6; ++i)
        {
            if (rState.gaitToNow[i] == MDT::SWING_FLAG)
            {
                if (feasible_positions.leg[i].size() != 0)
                {
                    // 寻找最靠近默认位置的落足点
                    Eigen::Vector3d nF = HexapodParameter::norminalFoothold_B[i];
                    Vector3 default_p = rState.pose.getT_W_B() * nF;
                    VectorX dis2(feasible_positions.leg[i].size());
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    VectorX::Index maxRow;
                    dis2.minCoeff(&maxRow);
                    hexapodState2.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState2.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState2.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState2.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState2.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState2.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState2.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState2.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    Vector3 default_p1 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.2, nF.y(), nF.z());
                    Vector3 default_p2 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.1, nF.y(), nF.z());
                    // Vector3 default_p3 = rState.pose.getT_W_B() * Eigen::Vector3d(nF.x() + 0.5, nF.y()-0.5, nF.z());
                    

                    default_p = default_p1;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState1.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState1.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState1.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState1.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState1.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState1.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState1.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState1.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    default_p = default_p2;
                    for (int j = 0; j < (int)feasible_positions.leg[i].size(); ++j)
                    {
                        dis2(j) = (feasible_positions.leg[i][j] - default_p).norm();
                    }
                    dis2.minCoeff(&maxRow);
                    hexapodState3.feetPosition[i].x() = feasible_positions.leg[i][maxRow](0);
                    hexapodState3.feetPosition[i].y() = feasible_positions.leg[i][maxRow](1);
                    hexapodState3.feetPosition[i].z() = feasible_positions.leg[i][maxRow](2);
                    hexapodState3.feetNormalVector[i].x() = feasible_normalVector.leg[i].at(maxRow).x();
                    hexapodState3.feetNormalVector[i].y() = feasible_normalVector.leg[i].at(maxRow).y();
                    hexapodState3.feetNormalVector[i].z() = feasible_normalVector.leg[i].at(maxRow).z();
                    hexapodState3.maxNormalForce[i] = resultF.maxNormalF[i][maxRow];
                    hexapodState3.frcitionMu[i] = resultF.frcitionMu[i][maxRow];

                    hexapodState1.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                    hexapodState2.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;
                    hexapodState3.faultStateToNow[i] = MDT::NORMAL_LEG_FLAG;

                    MDT::Vector3 startP;
                    startP << rState.feetPosition[i].x(), rState.feetPosition[i].y(), rState.feetPosition[i].z();

                    MDT::Vector3 endP;
                    endP << hexapodState1.feetPosition[i].x(), hexapodState1.feetPosition[i].y(), hexapodState1.feetPosition[i].z();
                    
                    if(!COLLISION_CHECK::isSwingLegCollision(startP, endP,  mapData, i, bodyPositionBegin, bodyPositionEnd, bodyPoseBegin, bodyPoseEnd))
                    {
                        flag1 = false;
                    }

                    endP << hexapodState2.feetPosition[i].x(), hexapodState2.feetPosition[i].y(), hexapodState2.feetPosition[i].z();
                    if(!COLLISION_CHECK::isSwingLegCollision(startP, endP,  mapData, i, bodyPositionBegin, bodyPositionEnd, bodyPoseBegin, bodyPoseEnd))
                    {
                        flag2 = false;
                    }

                    endP << hexapodState3.feetPosition[i].x(), hexapodState3.feetPosition[i].y(), hexapodState3.feetPosition[i].z();
                    if(!COLLISION_CHECK::isSwingLegCollision(startP, endP, mapData, i, bodyPositionBegin, bodyPositionEnd, bodyPoseBegin, bodyPoseEnd))
                    {
                        flag3 = false;
                    }
                }
                else
                {
                    Eigen::Vector3d dafult_position = HexapodParameter::norminalFoothold_B[i];
                    dafult_position(2) += 0.3;
                    dafult_position = hexapodState1.pose.getT_W_B() * dafult_position;
                    hexapodState1.feetPosition[i].x() = dafult_position.x();
                    hexapodState1.feetPosition[i].y() = dafult_position.y();
                    hexapodState1.feetPosition[i].z() = dafult_position.z();

                    hexapodState2.feetPosition[i] = hexapodState1.feetPosition[i];  
                    hexapodState3.feetPosition[i] = hexapodState1.feetPosition[i];

                    hexapodState1.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                    hexapodState2.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                    hexapodState3.faultStateToNow[i] = MDT::FAULT_LEG_FLAG;
                }
            }
        }

        if(flag1)
        {
            resultLists.push_back(hexapodState1);
        }
        if(flag2)
        {
            resultLists.push_back(hexapodState2);
        }
        if(flag3)
        {
            resultLists.push_back(hexapodState3);
        }
        // resultLists.push_back(hexapodState1);
        // resultLists.push_back(hexapodState2);
        // resultLists.push_back(hexapodState3);

        return resultLists;
    }


    // 用于TreeNode::expansion
    std::vector<MDT::RobotState> getNextMCTSstateList_underConstrains_moreStates(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {
        std::vector<MDT::RobotState> resultLists;
        // std::vector<MDT::Vector6b> availableSupportState_ = PLANNING::findAvailableSupportStates(rState);
        // std::vector<float> maxStepLengthForSupportState = findAvailableStepLength(rState,availableSupportState_);

        auto result = PLANNING::getSupportListAndStepL_underConstrains(rState,mapData);
        std::vector<MDT::Vector6b> availableSupportState_ = result.first;

        if(availableSupportState_.size() == 0)
        {
            return resultLists;
        }

        assert(availableSupportState_.size() > 0 && "availableSupportState_ should not be empty.");

        std::vector<Eigen::Vector3d> maxMotion = result.second;
        std::vector<double> maxStepLengthForSupportState(maxMotion.size());
        std::transform(maxMotion.begin(), maxMotion.end(), maxStepLengthForSupportState.begin(), [](const Eigen::Vector3d& motion) {
            return motion.x();
        });

        std::vector<double> pitchMove(maxMotion.size());
        std::transform(maxMotion.begin(), maxMotion.end(), pitchMove.begin(), [](const Eigen::Vector3d& motion) {
            return motion.y();
        });


        // z轴方向的移动
        float averageFootholdsHeight = 0.0f;
        int count = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(rState.faultStateToNow[j] == MDT::NORMAL_LEG_FLAG)
            {
                count++;
                averageFootholdsHeight += rState.feetPosition[j].z();
            }
        }
        averageFootholdsHeight/=float(count);

        for(int i=0;i<availableSupportState_.size();i++)
        {

            MDT::RobotState hexapodState = rState;
            hexapodState.gaitToNow = availableSupportState_[i];
            float stepLength_ = maxStepLengthForSupportState[i] * 0.8f;


            hexapodState.pose.x += stepLength_ * cos(hexapodState.moveDirection);
            hexapodState.pose.y += stepLength_ * sin(hexapodState.moveDirection);
            
            // pitch方向的移动
            hexapodState.pose.pitch = pitchMove[i];
            hexapodState.maxNormalForce[0] = pitchMove[i];

            // z轴方向的移动
            float maxZ = hexapodState.pose.z + result.second[i].z();
            if(averageFootholdsHeight+USER::norminalTrunkHeight > maxZ)
            {
                hexapodState.pose.z = maxZ;
            }
            else
            {
                hexapodState.pose.z = averageFootholdsHeight + USER::norminalTrunkHeight;
            }

            


            // if(USER::COLLISION_CHECK)
            // {
            //     Vector3 bodyPositionBegin;
            //     bodyPositionBegin << rState.pose.x, rState.pose.y, rState.pose.z;
            //     Vector3 bodyPoseBegin;
            //     bodyPoseBegin << rState.pose.roll, rState.pose.pitch, rState.pose.yaw;
            //     auto results = swingLegFoot_position_lists_afterCollisionCheck(bodyPositionBegin, bodyPoseBegin, hexapodState, mapData);
            //     for(auto state_:results)
            //     {
            //         resultLists.push_back(state_);
            //     }
            // }
            // else

            auto results = swingLegFoot_position_lists(hexapodState, mapData, 0.1);
            for(auto state_:results)
            {
                if(USER::COLLISION_CHECK)
                {
                    bool collisionFlag = false;
                    for(int i = 0; i < 6; ++i)
                    {
                        if(state_.gaitToNow(i) == MDT::SWING_FLAG)
                        {
                            Eigen::Vector3d startP;
                            Eigen::Vector3d endP;
                            startP << hexapodState.feetPosition[i].x(), hexapodState.feetPosition[i].y(), hexapodState.feetPosition[i].z();
                            endP << state_.feetPosition[i].x(), state_.feetPosition[i].y(), state_.feetPosition[i].z();
                            bool flag_ = COLLISION_CHECK::isFootTrajectoryCollision(startP, endP, mapData);
                            if(!flag_)
                            {
                                collisionFlag = true;
                                break;
                            }
                        }
                    }

                    // 如果碰撞了，该状态直接忽略
                    if(collisionFlag)
                    {
                        continue;
                    }

                    resultLists.push_back(state_);
                }
                else
                {
                    resultLists.push_back(state_);
                }
                
            }


        }
        return resultLists;
    }

    // 用于TreeNode::expansion
    std::vector<MDT::RobotState> getNextMCTSstateByExpert_rotate_moreState(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {
        std::vector<MDT::RobotState> resultLists;

        float deltaYaw = rState.moveDirection - rState.pose.yaw;
        auto result = getSupportList_Rotate_underConstrains(rState, mapData);
        std::vector<MDT::Vector6b> availableSupportState_ = result.first;
        auto maxRotateAngleList = result.second;

        if(availableSupportState_.size() == 0)
        {
            std::cout<<"availableSupportState_.size() == 0"<<std::endl;
            return resultLists;
        }
        // z轴方向的移动
        float averageFootholdsHeight = 0.0f;
        int count = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(rState.faultStateToNow[j] == MDT::NORMAL_LEG_FLAG)
            {
                count++;
                averageFootholdsHeight += rState.feetPosition[j].z();
            }
        }
        averageFootholdsHeight/=float(count);
        


        for(int i=0;i<availableSupportState_.size();i++)
        {

            MDT::RobotState hexapodState = rState;
            hexapodState.gaitToNow = availableSupportState_[i];
            float maxRotateAngle_ = maxRotateAngleList[i];

            // 根据最大可行旋转角度和目标角度差,确定最终旋转角度增量
            float deltaAngle = 0.0f;
            if(fabs(deltaYaw) < fabs(maxRotateAngle_))
            {
                deltaAngle = deltaYaw;
            }
            else
            {
                deltaAngle = maxRotateAngle_;
            }
            hexapodState.pose.yaw += deltaAngle;
            hexapodState.pose.z = averageFootholdsHeight + USER::norminalTrunkHeight;

            auto results = swingLegFoot_position_lists(hexapodState, mapData, 0.0);
            for(auto state_:results)
            {
                resultLists.push_back(state_);
            }

        }


        return resultLists;
    }

    /**
     * \brief 根据当前状态 计算下一步可行接触状态 (挑选最大步长步态)  用于单步规划测试
     * \param hexapodState 机器人当前状态
     * \param mapData  地图数据
     * \return 返回下一步可行接触状态
    */
    MDT::RobotState getNextMCTSstateByExpert_underConstrain(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {

        auto result = PLANNING::getSupportListAndStepL_underConstrains(rState, mapData);
        std::vector<MDT::Vector6b> availableSupportState_ = result.first;

        if(availableSupportState_.size() == 0)
        {
            std::cout<<"availableSupportState_.size() == 0"<<std::endl;
            return rState;
        }

        assert(availableSupportState_.size() > 0 && "availableSupportState_ should not be empty.");

        std::vector<Eigen::Vector3d> maxMotion = result.second;
        std::vector<double> maxStepLengthForSupportState(maxMotion.size());
        std::transform(maxMotion.begin(), maxMotion.end(), maxStepLengthForSupportState.begin(), [](const Eigen::Vector3d& motion) {
            return motion.x();
        });



        //获得maxStepLengthForSupportState中最大元素的索引
        int maxIndex = std::max_element(maxStepLengthForSupportState.begin(), maxStepLengthForSupportState.end()) - maxStepLengthForSupportState.begin();
        MDT::RobotState hexapodState = rState;
        hexapodState.gaitToNow = availableSupportState_[maxIndex];
        float stepLength_ = maxStepLengthForSupportState[maxIndex]*2.0/3.0;

        std::cout << "************ index : " << maxIndex << std::endl;

        if(stepLength_ > 0.05f)
        {
            // 防止落在支撑多边形边上
            stepLength_ -= 0.05f;
        }
        else if(stepLength_ < 0.01) //防止在仿真过程中机器人不断挪动，由于数值问题移动出扇形工作空间
        {
            stepLength_ = 0;
        }
        

        hexapodState.pose.x += stepLength_ * cos(hexapodState.moveDirection);
        hexapodState.pose.y += stepLength_ * sin(hexapodState.moveDirection);
        // z轴方向的移动
        float averageFootholdsHeight = 0.0f;
        int count = 0;
        for(int j = 0; j < 6; ++j)
        {
            if(rState.faultStateToNow[j] == MDT::NORMAL_LEG_FLAG)
            {
                count++;
                averageFootholdsHeight += rState.feetPosition[j].z();
            }
        }
        averageFootholdsHeight/=float(count);
        hexapodState.pose.z = averageFootholdsHeight + USER::norminalTrunkHeight;

        //findAvailableFootholdsCombination
        //findRandomFootholdsCombination

        // if(PLANNING::swingLegFoot_position_Expert_forSim2(hexapodState, mapData)) 
        // {
        //     return hexapodState;
        // }
        // grid_map::SignedDistanceField sdf(mapData, "elevation", -1, 3);

        Vector3 bodyPositionBegin;
        bodyPositionBegin << rState.pose.x, rState.pose.y, rState.pose.z;

        Vector3 bodyPoseBegin;
        bodyPoseBegin << rState.pose.roll, rState.pose.pitch, rState.pose.yaw;

        std::vector<MDT::RobotState> resultLists = swingLegFoot_position_lists(hexapodState, mapData, 0.1);

        if(resultLists.size() == 0)
        {
            std::cout << "resultLists.size() == 0" << std::endl;
            exit(0);
        }

        int randIndex = rand() % resultLists.size();
        return resultLists[randIndex];

    }



    MDT::RobotState getNextMCTSstateByExpert_rotate(const MDT::RobotState rState, const grid_map::GridMap &mapData)
    {
        float deltaYaw = rState.moveDirection - rState.pose.yaw;
        auto result = getSupportList_Rotate_underConstrains(rState, mapData);
        std::vector<MDT::Vector6b> availableSupportState_ = result.first;
        auto maxRotateAngleList = result.second;

        if(availableSupportState_.size() == 0)
        {
            std::cout<<"availableSupportState_.size() == 0"<<std::endl;
            return rState;
        }


        int targetIndex = 0;
        // 获得最大旋转量的索引
        if(deltaYaw < 0)
        {
            targetIndex = std::min_element(maxRotateAngleList.begin(), maxRotateAngleList.end()) - maxRotateAngleList.begin();
        }
        else
        {
            targetIndex = std::max_element(maxRotateAngleList.begin(), maxRotateAngleList.end()) - maxRotateAngleList.begin();
        }


        MDT::RobotState hexapodState = rState;
        hexapodState.gaitToNow = availableSupportState_[targetIndex];
        float maxRotateAngle_ = maxRotateAngleList[targetIndex];

        // 根据最大可行旋转角度和目标角度差,确定最终旋转角度增量
        float deltaAngle = 0.0f;
        if(fabs(deltaYaw) < fabs(maxRotateAngle_))
        {
            deltaAngle = deltaYaw;
        }
        else
        {
            deltaAngle = maxRotateAngle_;
        }



        hexapodState.pose.yaw += deltaAngle;

        std::vector<MDT::RobotState> resultLists = swingLegFoot_position_lists(hexapodState, mapData, 0.0);

        if(resultLists.size() == 0)
        {
            std::cout << "resultLists.size() == 0" << std::endl;
            exit(0);
        }

        int randIndex = rand() % resultLists.size();
        return resultLists[randIndex];

    }



    //已知当前的机体位置，以及待跟踪的路径序列，得到当前机体应该前进的目标点
    // 倒着序列找，找到第一个距离机体位置小于阈值的点
    int getTrackMovingDestination(Eigen::Vector3f robotPos, const std::vector<Eigen::Vector3f>& pathPnts){
        float threshold = 0.4;
        int resultIndex = 0;
        bool isFind = false;

        if (!pathPnts.empty()) {
            for (int i = pathPnts.size() - 1; i >= 0; --i) {
                float xx = pathPnts[i].x() - robotPos.x();
                float yy = pathPnts[i].y() - robotPos.y();
                float zz = 0;

                if (sqrt(xx * xx + yy * yy + zz * zz) < threshold) {
                    resultIndex = i;
                    isFind = true;
                    break;
                }
            }
        }

        if (!isFind)
        {
            resultIndex = pathPnts.size() - 1;
        }
        
        return resultIndex;
    }

    

    //得到当前应该前进的方向的单位向量
    Eigen::Vector3f getTrackMovingDirection(Eigen::Vector3f robotPos, const std::vector<Eigen::Vector3f>& pathPnts){
        auto index_ = getTrackMovingDestination(robotPos,pathPnts);
        Eigen::Vector3f destinationPoint = pathPnts[index_];

        //防止机器人临近最后一路径点时, 计算的方向发生巨大改变.
        if(index_ == pathPnts.size() - 1)
        {
            Eigen::Vector3f vectorDir = pathPnts[index_ - 1] - pathPnts[index_ - 2];
            return vectorDir.normalized();
        }

        Eigen::Vector3f result;
        result.x() = destinationPoint.x() - robotPos.x();
        result.y() = destinationPoint.y() - robotPos.y();
        result.z() = 0;

        // std::cout << "destinationPoint: " << destinationPoint << std::endl;
        // std::cout << "robotPos: " << robotPos << std::endl;

        result.normalize();
        
        // float length = sqrt(result.x*result.x + result.y*result.y + result.z*result.z);
        // result.x /= length;
        // result.y /= length;
        // result.z /= length;
        return result;
    }

    int findTheNearestPathPointIndex(Eigen::Vector3f robotPos, const std::vector<Eigen::Vector3f>& pathPnts)
    {
        std::vector<float> disList;
        for(int iii = 0;iii<pathPnts.size();iii++){
            float xx = pathPnts[iii].x() - robotPos.x();
            float yy = pathPnts[iii].y() - robotPos.y();
            disList.push_back(sqrt(xx*xx + yy*yy));
        }

        int minIndex = min_element(disList.begin(),disList.end()) - disList.begin();
        return minIndex;
    }


    bool isOverAngle(MatrixXX* A_output_B, VectorX* b_output_B, MDT::POINT foot_B, int legIndex) {
        if(!Robot_State_Transition::isInConvex( A_output_B[legIndex],  b_output_B[legIndex], foot_B))
        {
            return false; // 质心位置不在运动学凸包内部
        }
        return true;
    }

    // 使用二分法确定最大旋转角度
    std::pair<double, double> binarySearchMaxRotationAngle(MatrixXX* A_output_B, VectorX* b_output_B, MDT::POINT foot_B, int legIndex) {
        double negativeYaw = 0.0f;
        double low = 0.0;
        double high = M_PI/2; 

        // 二分法查找
        while (high - low > 0.001) {
            double mid = (low + high) / 2;

            // 构建旋转矩阵
            Eigen::AngleAxisd rotation(mid, Eigen::Vector3d::UnitZ());  //注意,这里只是绕着机器人足端所在平面的法线旋转, 如果非法线,则不适用.

            // 进行旋转
            Eigen::Vector3d feet_Rotate = rotation * foot_B;

            // 在左半边搜索
            if (isOverAngle(A_output_B, b_output_B, feet_Rotate, legIndex)) {
                low = mid;
            } else {
                high = mid;
            }
        }
        negativeYaw = -low;



        double positiveYaw = 0.0f;
        low = 0.0;
        high = M_PI/2;
        // 二分法查找
        while (high - low > 0.001) {
            double mid = (low + high) / 2;

            // 构建旋转矩阵
            Eigen::AngleAxisd rotation(-mid, Eigen::Vector3d::UnitZ());  //注意,这里只是绕着机器人足端所在平面的法线旋转, 如果非法线,则不适用.

            // 进行旋转
            Eigen::Vector3d feet_Rotate = rotation * foot_B;

            // 在左半边搜索
            if (isOverAngle(A_output_B, b_output_B, feet_Rotate, legIndex)) {
                low = mid;
            } else {
                high = mid;
            }
        }
        positiveYaw = low;

        return std::make_pair(negativeYaw, positiveYaw);

    }



    // 获取机器人满足运动学约束的最大旋转量
    //注意,这里只是绕着机器人足端所在平面的法线旋转,角度最大值, 如果非法线,则不适用.
    std::pair<double, double> getMaxYawRotationAngle(MDT::RobotState currentState)
    {
        // 设置se3为原点的位姿,以获得在Body坐标系下的足端约束不等式系数
        MatrixXX A_output[6];
        VectorX b_output[6];
        MDT::Pose ppose;
        ppose.x = 0;
        ppose.y = 0;
        ppose.z = 0;
        ppose.roll = 0;
        ppose.pitch = 0;
        ppose.yaw = 0;
        Robot_State_Transition::get_kinematics_con_cog_foot(ppose, A_output, b_output);

        //根据当前足端在Body坐标系下的位置 和 运动学约束不等式, 求解最大旋转角度.
        auto T_B_W = currentState.pose.getT_W_B().inverse();
        MDT::POINT feetB[6];
        for(int i=0;i<6;++i)
        {
            feetB[i] = T_B_W * currentState.feetPosition[i];
        }
        std::vector<double> negativeAngleList;
        std::vector<double> positiveAngleList;

        for (int i = 0; i < 6; i++)
        {
            if(currentState.gaitToNow[i] == MDT::SWING_FLAG)
            {
                continue;
            }
            auto yawAngleRange = binarySearchMaxRotationAngle(A_output, b_output, feetB[i], i);
            negativeAngleList.push_back(yawAngleRange.first);
            positiveAngleList.push_back(yawAngleRange.second);
            // std::cout << "leg: " << i << " negativeAngle: " << yawAngleRange.first << " positiveAngle: " << yawAngleRange.second << std::endl;
        }
        
        double negativeYaw = *std::max_element(negativeAngleList.begin(), negativeAngleList.end());
        double positiveYaw = *std::min_element(positiveAngleList.begin(), positiveAngleList.end());

        return std::make_pair(negativeYaw, positiveYaw);
    }

    std::pair<float, float> getTargetYawAndMoveDir(MDT::RobotState currentState, const std::vector<Eigen::Vector3f>& pathPnts)
    {

        // 根据路径和机器人当前位置,确定机器人前进方向
        auto dirVector = getTrackMovingDirection(Eigen::Vector3f(currentState.pose.x, currentState.pose.y, currentState.pose.z), pathPnts);
        float moveDirAngle = atan2(dirVector.y(), dirVector.x());

        // if(moveDirAngle < 0)
        // {
        //     moveDirAngle += 2*M_PI;
        // }

        // if(moveDirAngle > M_PI)
        // {
        //     std::cout << "dirVector: " << dirVector.transpose() << std::endl;
        //     auto index_ = getTrackMovingDestination(Eigen::Vector3f(currentState.pose.x, currentState.pose.y, currentState.pose.z),pathPnts);
        //     Eigen::Vector3f destinationPoint = pathPnts[index_];

        //     std::cout << "destinationPoint: " << destinationPoint.transpose() << std::endl;
        //     std::cout << "currentState: " << currentState.pose.x << ", " << currentState.pose.y << ", " << currentState.pose.z << std::endl;
        //     exit(0);
        // }


        // if(currentState.pose.yaw < 0)
        // {
        //     currentState.pose.yaw += 2*M_PI;
        // }

        currentState.moveDirection = moveDirAngle; 

        // 临时这么弄,这里还需完善
        currentState.gaitToNow << MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG, MDT::SUPPORT_FLAG;
        // 根据机器人当前位置和运动学约束,确定机器人最大旋转角度
        auto yawAngleRange = getMaxYawRotationAngle(currentState);

        float deltaAngle = 0.0f;

        float deltaYaw = moveDirAngle - currentState.pose.yaw;

        // 根据最大可行旋转角度和目标角度差,确定最终旋转角度增量
        if(deltaYaw < 0)
        {
            if(fabs(deltaYaw) < fabs(yawAngleRange.first))
            {
                deltaAngle = deltaYaw;
            }
            else
            {
                deltaAngle = yawAngleRange.first;
            }
        }
        else
        {
            if(fabs(deltaYaw) < fabs(yawAngleRange.second))
            {
                deltaAngle = deltaYaw;
            }
            else
            {
                deltaAngle = yawAngleRange.second;
            }
        }

        // std::cout << "moveDirAngle: " << moveDirAngle/3.1415926*180 << std::endl;
        // std::cout << "deltaYaw: " << deltaYaw/3.1415926*180 << std::endl;
        // std::cout << "yawAngleRange-: " << yawAngleRange.first/3.1415926*180 << std::endl;
        // std::cout << "yawAngleRange+: " << yawAngleRange.second/3.1415926*180 << std::endl;
        // std::cout << "deltaAngle: " << deltaAngle/3.1415926*180 << std::endl;
        // std::cout << "current yawAngle: " << currentState.pose.yaw/3.1415926*180 << std::endl;

        float resultYaw = currentState.pose.yaw + deltaAngle;

        return std::make_pair(resultYaw, moveDirAngle);


    }



    float getTwoStateDistanceAlongPath(const MDT::RobotState &startState, const MDT::RobotState &endState, const std::vector<Eigen::Vector3f>& pathPnts)
    {
        Eigen::Vector3f robotPos = {startState.pose.x, startState.pose.y, startState.pose.z};
        auto startPntIndex = PLANNING::findTheNearestPathPointIndex(robotPos, pathPnts);
        
        Eigen::Vector3f robotPos2 = {endState.pose.x, endState.pose.y, endState.pose.z};
        auto endPntIndex = PLANNING::findTheNearestPathPointIndex(robotPos2, pathPnts);

        //　计算仿真距离
        float simDis = 0.0;
        if(startPntIndex > endPntIndex)
        {
            std::cout << "startState:" << robotPos.transpose() << std::endl;
            std::cout << "endState:" << robotPos2.transpose() << std::endl;
            std::cout << "startPntIndex:" << startPntIndex << ", " << "endPntIndex:" << endPntIndex << std::endl;
            std::cout << "startPntIndex > endPntIndex" << std::endl;
            exit(0);
        }
        for(int i = startPntIndex; i <= endPntIndex; ++i)
        {
            simDis += (pathPnts[i+1] - pathPnts[i]).norm();
        }

        return simDis;
    }

}