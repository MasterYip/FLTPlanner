#ifndef CUSTOM_VARIABLE_H
#define CUSTOM_VARIABLE_H
#include <Eigen/Dense>
#include <iostream>
#include <vector> //STL容器

namespace MDT
{
#define _PI_ 3.14159265358979323846
    // #define MDT::SUPPORT_FLAG 0
    // #define MDT::SWING_FLAG 1
    // #define MDT::FAULT_LEG_FLAG 1
    // #define MDT::NORMAL_LEG_FLAG 0

    const int SUPPORT_FLAG = 0;
    const int SWING_FLAG = 1;
    const int FAULT_LEG_FLAG = 1;
    const int NORMAL_LEG_FLAG = 0;

    //------数据类型的定义------
    typedef Eigen::Vector3d POINT;
    typedef Eigen::Array<bool, 6, 1> Vector6b;                                                   // 用于存储六个布尔值的容器. 表示支撑状态 和 容错状态
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d; // 用于存储三维向量的容器
    typedef Eigen::Matrix<double, 3, 1> Vector3;                                                 // 三维点

    // 备选落足点集合
    struct VectorList
    {
        std::vector<Vector3, Eigen::aligned_allocator<Vector3>> leg[6];
    };

    struct AvailableContactsInfo
    {
        VectorList position;
        VectorList normalVector;
        std::vector<float> frcitionMu[6];
        std::vector<float> maxNormalF[6];
    };

    MDT::POINT pointRotationAndTrans(const MDT::POINT &pnt_currentF, const Eigen::Isometry3d &T_targetF_currentF);
    MDT::POINT pointRotation(const MDT::POINT &pnt_currentF, const Eigen::Matrix3d &R_targetF_currentF);

    struct FeetPositions
    {
        POINT footP[6];
    };

    struct FootholdsData
    {
        POINT footP[6];
        POINT normalV[6];
    };

    /**
     * @brief 机器人位姿
     */
    struct Pose
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;

        POINT getTrunkPos() const
        {
            POINT point;
            point << x, y, z;
            return point;
        }

        Eigen::Matrix3d to_rotation_matrix() const
        {
            Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle; // 按照z y x （EULAR ZYX的顺序变换的）
            return q.toRotationMatrix();
        }

        // 返回旋转矩阵
        Eigen::Isometry3d getT_W_B() const
        {
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
            Eigen::Isometry3d matrix = Eigen::Isometry3d::Identity();
            matrix.rotate(quaternion);
            matrix.translation() << x, y, z;
            return matrix;
        }
    };

    /**
     * @brief 机器人状态
     */
    struct RobotState
    {
        Pose pose; // roll,yaw,pitch单位为弧度
        Vector6b gaitToNow;
        Vector6b faultStateToNow;
        POINT feetPosition[6];
        POINT feetNormalVector[6];
        float maxNormalForce[6];
        float frcitionMu[6];
        float moveDirection; // 单位为弧度

        bool supportFlag() const
        {
            return MDT::SUPPORT_FLAG;
        }

        bool swingFLag() const
        {
            return MDT::SWING_FLAG;
        }

        bool faultFlag() const
        {
            return MDT::FAULT_LEG_FLAG;
        }

        void initialize()
        {
            pose.x = 0;
            pose.y = 0;
            pose.z = 0;
            pose.roll = 0;
            pose.pitch = 0;
            pose.yaw = 0;

            for (int i = 0; i < 6; ++i)
            {
                gaitToNow[i] = 0;
                faultStateToNow[i] = 0;
                feetPosition[i].x() = 0;
                feetPosition[i].y() = 0;
                feetPosition[i].z() = 0;
                feetNormalVector[i] << 0, 0, 1;
                maxNormalForce[i] = 1000.0f;
                frcitionMu[i] = 0.8f;
            }
        }
    };

    // /**
    //  * @brief 机器人位姿重载输出流
    // */
    // std::ostream& operator<<(std::ostream& os, const Pose& pose)
    // {
    //     os << "x: " << pose.x << "; y: " << pose.y << "; z: " << pose.z << "; roll: " << pose.roll << "; pitch: " << pose.pitch << "; yaw: " << pose.yaw;
    //     return os;
    // }

    // /**
    //  * @brief 机器人状态重载输出流
    // */
    // std::ostream& operator<<(std::ostream& os, const RobotState& robotState)
    // {
    //     os << "Pose: " << robotState.pose << std::endl;
    //     os << "Gait to now: ";
    //     for (int i = 0; i < 6; ++i)
    //     {
    //         os << robotState.gaitToNow[i] << " ";
    //     }
    //     os << std::endl;
    //     os << "Fault state to now: ";
    //     for (int i = 0; i < 6; ++i)
    //     {
    //         os << robotState.faultStateToNow[i] << " ";
    //     }
    //     os << std::endl;
    //     os << "Feet position: ";
    //     for (int i = 0; i < 6; ++i)
    //     {
    //         os << "(" << robotState.feetPosition[i].x() << "," << robotState.feetPosition[i].y() << "," << robotState.feetPosition[i].z() << ") ";
    //     }
    //     os << std::endl;
    //     os << "Support flag: " << robotState.supportFlag() << std::endl;
    //     os << "Swing flag: " << robotState.swingFLag() << std::endl;
    //     return os;
    // }

} // namespace name

#endif
