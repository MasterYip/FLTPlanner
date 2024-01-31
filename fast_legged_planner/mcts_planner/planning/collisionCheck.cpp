#include <collisionCheck.h> 
#include <HexapodParameter.h>

const double SIN_15 = 0.25881904510252074;
const double COS_15 = 0.9659258262890683;
const double TAN_15 = 0.2679491924311227;

namespace COLLISION_CHECK{

    // Elspider的逆运动学求解
    Eigen::Vector3d InverseKinematics(const Eigen::Isometry3d T_W_B, const Eigen::Vector3d &World_foot, int legNum)
    {
        // 计算足端目标机器人坐标系到世界坐标系的旋转平移矩阵T
        // Eigen::Isometry3d T_W_B = getTrans_W_B(Wolrd_BasePose);

        // 计算足端到固定基节坐标系下的坐标
        Eigen::Vector3d p1_ = Eigen::Vector3d(World_foot.x(), World_foot.y(), World_foot.z());
        Eigen::Vector3d foot_FixJi;
        foot_FixJi = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[legNum] * T_W_B.inverse() * p1_;

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
            // debug
            std::cout << "foot_FixJi = " << foot_FixJi << std::endl;
            // ROS_ERROR("out of range,M:%f,N:%f", M, N);
        }
        float tmp_acos = acos((M * M + N * N) / sqrt(M * M + N * N));
        float tmp_theta2 = atan2(N, M) + tmp_acos;
        float tmp_theta3 = atan2(N - 0.5 * sin(tmp_theta2), M - 0.5f * cos(tmp_theta2)) - tmp_theta2;


        Eigen::Vector3d JointAngles(tmp_theta1, tmp_theta2, tmp_theta3);

        return JointAngles;
    }


    Eigen::Matrix4d matrixT(double theta, double afa, double l, double d) {
        Eigen::Matrix4d D;

        D << std::cos(theta), -std::sin(theta), 0, l,
            std::cos(afa) * std::sin(theta), std::cos(afa) * std::cos(theta), -std::sin(afa), -d * std::sin(afa),
            std::sin(afa) * std::sin(theta), std::sin(afa) * std::cos(theta), std::cos(afa), d * std::cos(afa),
            0, 0, 0, 1;

        return D;
    }

    // 通过正运动学插值腿部的碰撞检测点位置
    std::vector<Eigen::Vector3d> getLegKeyPoints(Eigen::Isometry3d T_W_B, Eigen::Vector3d EndEffectorP_W, int legNum)
    {
        // std::cout << "T_W_B: " << T_W_B.matrix() << std::endl;
        // std::cout << "EndEffectorP_W: " << EndEffectorP_W << std::endl;

        const Eigen::Vector3d JointAngles = InverseKinematics(T_W_B, EndEffectorP_W, legNum);

        // 正运动学部分
        Eigen::Matrix4d T_B_0 = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[legNum].inverse().matrix();
        Eigen::Matrix4d T_0_1 = matrixT(JointAngles(0), 0, 0, 0);
        Eigen::Matrix4d T_1_2 = matrixT(JointAngles(1), 3.1415926f/2.0f, 0.18, 0);
        Eigen::Matrix4d T_2_3 = matrixT(JointAngles(2), 0, 0.5, 0);
        Eigen::Matrix4d T_3_4 = matrixT(0,0,0.5,0);

        Eigen::Matrix4d T_W_3 = T_W_B * T_B_0 * T_0_1 * T_1_2 * T_2_3;

        Eigen::Matrix4d T1 = matrixT(0,0,0.2,0);
        Eigen::Matrix4d T2 = matrixT(0,0,0.3,0);
        Eigen::Matrix4d T3 = matrixT(0,0,0.4,0);

        // block<3, 1>(0, 3)提取了一个3x1的块，从复合变换矩阵的第0行、第3列开始，这个块包含了最终3D点的坐标
        std::vector<Eigen::Vector3d> endPs;

        endPs.push_back((T_W_3 * T1).block<3, 1>(0, 3));
        endPs.push_back((T_W_3 * T2).block<3, 1>(0, 3));
        endPs.push_back((T_W_3 * T3).block<3, 1>(0, 3));
        //  = (T_W_B * T_B_0 * T_0_1 * T_1_2 * T_2_3).block<3, 1>(0, 3);

        return endPs;
    }



    // 机体参数计算(五次多项式)  6：机体每个方向的6个解析参数  3：xyz三个方向
    std::pair< std::vector<Vector3>,  std::vector<Vector3> > getBodyTrajectory(const Vector3 startP, const Vector3 endP, const Vector3 poseBegin, const Vector3 poseEnd)
    {
        Eigen::Matrix<float, 6, 6> mat_body, inversemat_body;
        mat_body << 1, 0, 0, 0, 0, 0,         // t = 0位置
            1, 1, 1, 1, 1, 1,                 // t = 1位置
            0, 1, 0, 0, 0, 0,                 // t = 0速度
            0, 1, 2, 3, 4, 5,                 // t = 1速度
            0, 0, 2, 0, 0, 0,                 // t = 0加速度
            0, 0, 2, 6, 12, 20;               // t = 1加速度
        inversemat_body = mat_body.inverse(); // 矩阵求逆 Eigen库的用法
        Eigen::Matrix<float, 6, 3> solutionxyz_body;
        Eigen::Matrix<float, 6, 3> knownquantity_body = Eigen::MatrixXf::Zero(6, 3); // 矩阵初始化

        knownquantity_body(0, 0) = startP.x();
        knownquantity_body(0, 1) = startP.y();
        knownquantity_body(0, 2) = startP.z();
        knownquantity_body(1, 0) = endP.x();
        knownquantity_body(1, 1) = endP.y();
        knownquantity_body(1, 2) = endP.z();
        solutionxyz_body = inversemat_body * knownquantity_body;



        Eigen::Matrix<float, 6, 3> solutionbodyRotate;
        knownquantity_body(0, 0) = poseBegin.x();
        knownquantity_body(0, 1) = poseBegin.y();
        knownquantity_body(0, 2) = poseBegin.z();
        knownquantity_body(1, 0) = poseEnd.x();
        knownquantity_body(1, 1) = poseEnd.y();
        knownquantity_body(1, 2) = poseEnd.z();
        solutionbodyRotate = inversemat_body * knownquantity_body;

      
        std::vector<Vector3> keyPoints; 
        std::vector<Vector3> posePoints; 
        for(int ii=1; ii < 5; ii++)
        {
            float t = ii/6.0;
            Vector3 pnts_;
            pnts_(0) = solutionxyz_body(0, 0) + solutionxyz_body(1, 0) * t + solutionxyz_body(2, 0) * pow(t, 2) + solutionxyz_body(3, 0) * pow(t, 3) + solutionxyz_body(4, 0) * pow(t, 4) + solutionxyz_body(5, 0) * pow(t, 5);
            pnts_(1) = solutionxyz_body(0, 1) + solutionxyz_body(1, 1) * t + solutionxyz_body(2, 1) * pow(t, 2) + solutionxyz_body(3, 1) * pow(t, 3) + solutionxyz_body(4, 1) * pow(t, 4) + solutionxyz_body(5, 1) * pow(t, 5);
            pnts_(2) = solutionxyz_body(0, 2) + solutionxyz_body(1, 2) * t + solutionxyz_body(2, 2) * pow(t, 2) + solutionxyz_body(3, 2) * pow(t, 3) + solutionxyz_body(4, 2) * pow(t, 4) + solutionxyz_body(5, 2) * pow(t, 5);
            keyPoints.push_back(pnts_);

            pnts_(0) = solutionbodyRotate(0, 0) + solutionbodyRotate(1, 0) * t + solutionbodyRotate(2, 0) * pow(t, 2) + solutionbodyRotate(3, 0) * pow(t, 3) + solutionbodyRotate(4, 0) * pow(t, 4) + solutionbodyRotate(5, 0) * pow(t, 5);
            pnts_(1) = solutionbodyRotate(0, 1) + solutionbodyRotate(1, 1) * t + solutionbodyRotate(2, 1) * pow(t, 2) + solutionbodyRotate(3, 1) * pow(t, 3) + solutionbodyRotate(4, 1) * pow(t, 4) + solutionbodyRotate(5, 1) * pow(t, 5);
            pnts_(2) = solutionbodyRotate(0, 2) + solutionbodyRotate(1, 2) * t + solutionbodyRotate(2, 2) * pow(t, 2) + solutionbodyRotate(3, 2) * pow(t, 3) + solutionbodyRotate(4, 2) * pow(t, 4) + solutionbodyRotate(5, 2) * pow(t, 5);
            posePoints.push_back(pnts_);

        }

        return std::make_pair(keyPoints, posePoints);
    }






    std::vector<MDT::Vector3> getTrajectory(const MDT::Vector3 startP, const MDT::Vector3 endP)
    {
        Eigen::Matrix<float, 7, 7> mat_leg, inversemat_leg;
        mat_leg << 1, 0, 0, 0, 0, 0, 0,                                              // t = 0的位置
            1, 0.5, pow(0.5, 2), pow(0.5, 3), pow(0.5, 4), pow(0.5, 5), pow(0.5, 6), // t = 0.5的位置
            1, 1, 1, 1, 1, 1, 1,                                                     // t = 1的位置
            0, 1, 0, 0, 0, 0, 0,                                                     // t = 0的速度，为零
            0, 1, 2, 3, 4, 5, 6,                                                     // t = 1的速度，为零
            0, 0, 2, 0, 0, 0, 0,                                                     // t = 0的加速度，为零
            0, 0, 2, 6, 12, 20, 30;                                                  // t = 1的加速度，为零
        inversemat_leg = mat_leg.inverse();                                          // 矩阵求逆 Eigen库的用法

        Eigen::Matrix<float, 7, 3> solutionxyz = Eigen::MatrixXf::Zero(7, 3); // 矩阵初始化
        Eigen::Matrix<float, 7, 3> knownquantity = Eigen::MatrixXf::Zero(7, 3);
        for (int i = 0; i < 1; ++i)
        {
            knownquantity(0, 3 * i) = startP.x();
            knownquantity(0, 3 * i + 1) = startP.y();
            knownquantity(0, 3 * i + 2) = startP.z();
            knownquantity(2, 3 * i) = endP.x();
            knownquantity(2, 3 * i + 1) = endP.y();
            knownquantity(2, 3 * i + 2) = endP.z();

            float h = 0.1; // 摆动腿步高 ////////////////////////////////

            // 中间最高点的位置
            knownquantity(1, 3 * i) = (startP.x() + endP.x()) / 2.0f - h * SIN_15;
            knownquantity(1, 3 * i + 1) = (startP.y() + endP.y()) / 2.0f;
            knownquantity(1, 3 * i + 2) = (startP.z() + endP.z()) / 2.0f + h * COS_15;
        }
        // std::cout << knownquantity << std::endl;
        solutionxyz = inversemat_leg * knownquantity;


        std::vector<MDT::Vector3> keyPoints; 
        for(int ii=1; ii < 5; ii++)
        {
            float t = ii/6.0;
            MDT::Vector3 pnts_;
            pnts_(0) = solutionxyz(0, 3 * 0) + solutionxyz(1, 3 * 0) * t + solutionxyz(2, 3 * 0) * pow(t, 2) + solutionxyz(3, 3 * 0) * pow(t, 3) + solutionxyz(4, 3 * 0) * pow(t, 4) + solutionxyz(5, 3 * 0) * pow(t, 5) + solutionxyz(6, 3 * 0) * pow(t, 6);
            pnts_(1) = solutionxyz(0, 3 * 0 + 1) + solutionxyz(1, 3 * 0 + 1) * t + solutionxyz(2, 3 * 0 + 1) * pow(t, 2) + solutionxyz(3, 3 * 0 + 1) * pow(t, 3) + solutionxyz(4, 3 * 0 + 1) * pow(t, 4) + solutionxyz(5, 3 * 0 + 1) * pow(t, 5) + solutionxyz(6, 3 * 0 + 1) * pow(t, 6);
            pnts_(2) = solutionxyz(0, 3 * 0 + 2) + solutionxyz(1, 3 * 0 + 2) * t + solutionxyz(2, 3 * 0 + 2) * pow(t, 2) + solutionxyz(3, 3 * 0 + 2) * pow(t, 3) + solutionxyz(4, 3 * 0 + 2) * pow(t, 4) + solutionxyz(5, 3 * 0 + 2) * pow(t, 5) + solutionxyz(6, 3 * 0 + 2) * pow(t, 6);
            keyPoints.push_back(pnts_);
        }

        return keyPoints;
    }

    float getVirtualSDF(grid_map::GridMap map_, grid_map::Position3 pnt_)
    {
        float sdfV_ = 0.0f;

        grid_map::Index index_;
        grid_map::Position tmpP(pnt_(0), pnt_(1));

        map_.getIndex(tmpP, index_);
        sdfV_ = map_.at("elevation", index_);

        return pnt_(2) - sdfV_;
    }


    bool isSwingLegCollision(Eigen::Vector3d startP, Eigen::Vector3d endP, const grid_map::GridMap &mapData, int legNum,
        Eigen::Vector3d &BodyPositionBegin,  Eigen::Vector3d &BodyPositionEnd, Eigen::Vector3d &BodyPoseBegin, Eigen::Vector3d &BodyPoseEnd)
    {
        std::vector<Eigen::Vector3d> keyPnts;
        // std::vector<MDT::Vector3> keyPnts =  getTrajectory(startP, endP);
        auto results = getBodyTrajectory(BodyPositionBegin, BodyPositionEnd, BodyPoseBegin, BodyPoseEnd);
        auto footKeyPoints = getTrajectory(startP, endP);
        
        for(int i=0; i< results.first.size(); i++)
        {
            MDT::Pose pose_;
            pose_.x = results.first[i](0);
            pose_.y = results.first[i](1);
            pose_.z = results.first[i](2);
            pose_.roll = results.second[i](0);
            pose_.pitch = results.second[i](1);
            pose_.yaw = results.second[i](2);
            auto T_W_B = pose_.getT_W_B();
            auto aaP =  COLLISION_CHECK::getLegKeyPoints(T_W_B, footKeyPoints[i], legNum);

            for(int j=0; j< aaP.size(); j++)
            {
                keyPnts.push_back(aaP[j]);
            }
        }

        for( int i = 0; i< keyPnts.size(); i++)
        {
            // float sdfV = sdf.value(keyPnts[i]);
            float sdfV = getVirtualSDF(mapData, keyPnts[i]);
            if (sdfV <= 0.05)
            {
                std::cout << "collision index: " << i << std::endl;
                return false;
            }
        }
        return true;

    }

    // 只考虑足端轨迹的碰撞检测
    bool isFootTrajectoryCollision(Eigen::Vector3d startP, Eigen::Vector3d endP, const grid_map::GridMap &mapData)
    {
        auto footKeyPoints = getTrajectory(startP, endP);
        for( int i = 0; i< footKeyPoints.size(); i++)
        {
            // float sdfV = sdf.value(keyPnts[i]);
            float sdfV = getVirtualSDF(mapData, footKeyPoints[i]);
            if (sdfV <= 0.01)
            {
                std::cout << "collision index: " << i << std::endl;
                return false;
            }
        }
        return true;
    }

    bool isCollision_onePoint(Eigen::Vector3d pnt, const grid_map::GridMap &mapData)
    {
        float sdfV = getVirtualSDF(mapData, pnt);
        if (sdfV <= 0.05)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

}