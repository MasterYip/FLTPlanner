/*
 * @Author: XP
 */
#include "HexMini_IK_interface.hpp"
#include "ikfast_interface_RB.hpp"
#include "ikfast_interface_RF.hpp"
#include "ikfast_interface_LB.hpp"
#include "ikfast_interface_LF.hpp"

// 默认fixGu与body坐标系只有平移
void get_T_B_FixGu(Eigen::Isometry3d* T_B_FixGu)
{
    Eigen::Matrix<double, 6, 3> trans;

    // Gu节在body坐标系下的坐标
    trans.row(0) << 0.30, -0.08,  0.01100;  //RF
    trans.row(1) << 0.00, -0.14,  0.01100;  //RM
    trans.row(2) << -0.30, -0.08,  0.01100; //RB
    trans.row(3) << 0.30,  0.08,  0.01100;  //LF
    trans.row(4) << 0.00,  0.14,  0.01100;  //LM
    trans.row(5) << -0.30,  0.08,  0.01100; //LB

    for(int i=0;i<6;++i)
    {
        T_B_FixGu[i] = Eigen::Isometry3d::Identity();
        Eigen::Vector3d translation(trans(i,0), trans(i,1), trans(i,2));  // 例如，设置平移向量
        T_B_FixGu[i].translation() = translation;
    }

}


Eigen::Isometry3d getT_W_B(float x, float y, float z,float roll, float pitch, float yaw)
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

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getFeetP_fixGu_from_FeetP_W(float x, float y, float z,float roll, float pitch, float yaw,  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> feetPosition_W)
{
    Eigen::Isometry3d T_world_base = getT_W_B(x, y, z, roll, pitch, yaw);
    Eigen::Isometry3d T_B_FixGu[6];
    get_T_B_FixGu(T_B_FixGu);
    Eigen::Matrix<double, Eigen::Dynamic, 1> theta[18];

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> feetPosition_FixGu(6, 3);
    for(int i=0;i<6;++i)
    {
        Eigen::Matrix<double, 3, 1> p_W;
        p_W << feetPosition_W(i,0), feetPosition_W(i,1), feetPosition_W(i,2);

        Eigen::Matrix<double, 3, 1> p_B = T_world_base.inverse()*p_W; // 足端坐标在body坐标系下

        // 将足端坐标转换到FixGu坐标系下
        Eigen::Matrix<double, 3, 1> p_FixGu = T_B_FixGu[i].inverse()*p_B; // 足端坐标在FixGu坐标系下
        // std::cout << "p_FixGu: " << p_FixGu.transpose() << std::endl;
        feetPosition_FixGu.row(i) = p_FixGu;
    }
    return feetPosition_FixGu;
}


// 输入是每条腿在fixgu坐标系下的坐标, 求解
std::pair<bool, Eigen::Matrix<double, Eigen::Dynamic, 1>> fixGu_Frame_IK(Eigen::Matrix<double, 6, 3> initFeetPosition_FixGu)
{
    Eigen::Matrix<double, Eigen::Dynamic, 1> theta(18);
    for(int legN =0; legN<6; legN++)
    {
        std::vector<double> footP;
        // if(legN <3)
        // {
            footP.push_back(initFeetPosition_FixGu(legN,0));
            footP.push_back(initFeetPosition_FixGu(legN,1));
            footP.push_back(initFeetPosition_FixGu(legN,2));
        // }
        // else  // 左右对称和fixgu坐标系设置有关
        // {
        //     footP.push_back(-initFeetPosition_FixGu(legN,0));
        //     footP.push_back(-initFeetPosition_FixGu(legN,1));
        //     footP.push_back(initFeetPosition_FixGu(legN,2));
        // }

        std::vector<std::vector<double>> solret;
        // 后腿和前面的腿的求解方式不一样
        if(legN == 0 || legN == 1)
        {
            solret = IKFast_trans3D_RF(footP);
        }
        else if(legN == 2)
        {
            solret = IKFast_trans3D_RB(footP);
        }
        else if(legN == 3  || legN == 4)
        {
            solret = IKFast_trans3D_LF(footP);
        }
        else if(legN == 5)
        {
            solret = IKFast_trans3D_LB(footP);
        }

        if(solret.size() != 1)
        {
            // std::cout << "IKFast_trans3D_RB failed" << std::endl;
            // std::cout << "solret.size:" << solret.size() << std::endl;
            // std::cout << "legN:" << legN << std::endl;
            return std::make_pair(false, theta);
        }
        else
        {
            theta(legN*3) = solret[0][0];
            theta(legN*3+1) = solret[0][1];
            theta(legN*3+2) = solret[0][2];
        }

        
    }

    return std::make_pair(true, theta);
}


// 求解全身的逆运动学
std::pair<bool, Eigen::Matrix<double, Eigen::Dynamic, 1>> wholeBodyIK(float x_W, float y_W, float z_W,float roll, float pitch, float yaw,  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> feetPosition_W)
{
    auto inputGu = getFeetP_fixGu_from_FeetP_W(x_W, y_W, z_W, roll, pitch, yaw, feetPosition_W);
    auto result = fixGu_Frame_IK(inputGu);
    return result;
}

