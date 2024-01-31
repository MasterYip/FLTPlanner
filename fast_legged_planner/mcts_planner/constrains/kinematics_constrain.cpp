
#include "constrains/kinematics_constrain.hh"
#include "constrains/my_cdd.hh"
// #include "hit_spider/hexapod.hh"

namespace Robot_State_Transition
{

    std::pair<MatrixXX, VectorX> initHexMini_Ab_Ji_foot12()
    {
        MatrixXX AA;
        VectorX bb;
        MatrixX3 VV(10, 3);
        VV.setZero();
        VV.row(0) << 0.2412, -0.154, -0.1303;
        VV.row(1) << -0.07939, -0.1551, -0.1464;
        VV.row(2) << -0.0809, -0.1567, -0.3889;
        VV.row(3) << 0.2556, -0.1674, -0.3545;
        VV.row(4) << -0.3199, -0.3958, 0.006312;
        VV.row(5) << -0.2209, -0.2967, -0.3344;
        VV.row(6) << 0.3721, -0.2772, 0.02371;
        VV.row(7) << 0.3527, -0.2589, -0.2644;
        VV.row(8) << 0.05979, -0.4186, -0.2857;
        VV.row(9) << 0.06059, -0.472, 0.1195;

        bool flag = Robot_State_Transition::vertices_to_H(VV, AA, bb);
        if (!flag)
        {
            std::cout << "fail and exit" << std::endl;
            exit(0);
        }

        return std::make_pair(AA, bb);
    }

    std::pair<MatrixXX, VectorX> initHexMini_Ab_Ji_foot3()
    {
        MatrixXX AA;
        VectorX bb;
        MatrixX3 VV(10, 3);
        VV.setZero();
        VV.row(0) << 0.2412, -0.154, -0.1303;
        VV.row(1) << -0.07939, -0.1551, -0.1464;
        VV.row(2) << -0.0809, -0.1567, -0.3889;
        VV.row(3) << 0.2556, -0.1674, -0.3545;
        VV.row(4) << -0.3199, -0.3958, 0.006312;
        VV.row(5) << -0.2209, -0.2967, -0.3344;
        VV.row(6) << 0.3721, -0.2772, 0.02371;
        VV.row(7) << 0.3527, -0.2589, -0.2644;
        VV.row(8) << 0.05979, -0.4186, -0.2857;
        VV.row(9) << 0.06059, -0.472, 0.1195;
        for (int i = 0; i < VV.rows(); ++i) {
                VV(i, 0) = -VV(i, 0);
            }
        bool flag = Robot_State_Transition::vertices_to_H(VV, AA, bb);
        if (!flag)
        {
            std::cout << "fail and exit" << std::endl;
            exit(0);
        }

        return std::make_pair(AA, bb);
    }

    std::pair<MatrixXX, VectorX> initHexMini_Ab_Ji_foot45()
    {
        MatrixXX AA;
        VectorX bb;
        MatrixX3 VV(10, 3);
        VV.setZero();
        VV.row(0) << 0.2412, -0.154, -0.1303;
        VV.row(1) << -0.07939, -0.1551, -0.1464;
        VV.row(2) << -0.0809, -0.1567, -0.3889;
        VV.row(3) << 0.2556, -0.1674, -0.3545;
        VV.row(4) << -0.3199, -0.3958, 0.006312;
        VV.row(5) << -0.2209, -0.2967, -0.3344;
        VV.row(6) << 0.3721, -0.2772, 0.02371;
        VV.row(7) << 0.3527, -0.2589, -0.2644;
        VV.row(8) << 0.05979, -0.4186, -0.2857;
        VV.row(9) << 0.06059, -0.472, 0.1195;
        for (int i = 0; i < VV.rows(); ++i) {
                VV(i, 1) = -VV(i, 1);
            }
        bool flag = Robot_State_Transition::vertices_to_H(VV, AA, bb);
        if (!flag)
        {
            std::cout << "fail and exit" << std::endl;
            exit(0);
        }

        return std::make_pair(AA, bb);
    }

    std::pair<MatrixXX, VectorX> initHexMini_Ab_Ji_foot6()
    {
        MatrixXX AA;
        VectorX bb;
        MatrixX3 VV(10, 3);
        VV.setZero();
        VV.row(0) << 0.2412, -0.154, -0.1303;
        VV.row(1) << -0.07939, -0.1551, -0.1464;
        VV.row(2) << -0.0809, -0.1567, -0.3889;
        VV.row(3) << 0.2556, -0.1674, -0.3545;
        VV.row(4) << -0.3199, -0.3958, 0.006312;
        VV.row(5) << -0.2209, -0.2967, -0.3344;
        VV.row(6) << 0.3721, -0.2772, 0.02371;
        VV.row(7) << 0.3527, -0.2589, -0.2644;
        VV.row(8) << 0.05979, -0.4186, -0.2857;
        VV.row(9) << 0.06059, -0.472, 0.1195;
        for (int i = 0; i < VV.rows(); ++i) {
                VV(i, 1) = -VV(i, 1);
                VV(i, 0) = -VV(i, 0);
            }
        bool flag = Robot_State_Transition::vertices_to_H(VV, AA, bb);
        if (!flag)
        {
            std::cout << "fail and exit" << std::endl;
            exit(0);
        }

        return std::make_pair(AA, bb);
    }


    const auto Ab_Ji_foot_forHexMini12 = initHexMini_Ab_Ji_foot12();
    const auto Ab_Ji_foot_forHexMini3 = initHexMini_Ab_Ji_foot3();
    const auto Ab_Ji_foot_forHexMini45 = initHexMini_Ab_Ji_foot45();
    const auto Ab_Ji_foot_forHexMini6 = initHexMini_Ab_Ji_foot6();
    // const VectorX b_Ji_foot_forHexMini = initHexMini_b_Ji_foot();
    void get_kinematics_con_cog_foot(const MDT::Pose &base_pose, MatrixXX *A_output, VectorX *b_output)
    {
        // base描述world的齐次变换矩阵
        Eigen::Isometry3d T_world_base = base_pose.getT_W_B(); 
        Matrix3 base_world_R = T_world_base.matrix().block(0, 0, 3, 3).transpose();
        Vector3 base_world_p = -base_world_R * T_world_base.matrix().block(0, 3, 3, 1);

        for (int i = 0; i < 6; ++i)
        {
            // fixJi坐标系描述base
            Matrix3 Ji_base_R = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[i].matrix().block(0, 0, 3, 3);
            Vector3 Ji_base_p = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[i].matrix().block(0, 3, 3, 1);
            MatrixXX A_foot_Ji;
            VectorX b_foot_Ji;


            // 增加约束,防止腿部工作空间交叉
            switch (i)
            {
                case 0:
                    A_foot_Ji = Ab_Ji_foot_forHexMini12.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini12.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.00;
                    break;

                case 1:
                    A_foot_Ji = Ab_Ji_foot_forHexMini12.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini12.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-0.11;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.1;
                    break;
                
                case 2:
                    A_foot_Ji = Ab_Ji_foot_forHexMini3.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini3.second;
                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-  0.25;;
                break;

                case 3:
                    A_foot_Ji = Ab_Ji_foot_forHexMini45.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini45.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.00;
                break;

                case 4:
                    A_foot_Ji = Ab_Ji_foot_forHexMini45.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini45.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-0.11;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.1;
                break;

                case 5:
                    A_foot_Ji = Ab_Ji_foot_forHexMini6.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini6.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-  0.25;;
                break;

                default:
                    break;
            }

            A_output[i].resize(A_foot_Ji.rows(), 3);
            b_output[i].resize(A_foot_Ji.rows(), 1);

            auto a = A_foot_Ji * Ji_base_R * base_world_R;
            auto b = b_foot_Ji - A_foot_Ji * (Ji_base_R * base_world_p + Ji_base_p);
            A_output[i] = a;
            b_output[i] = b;

        }

    }


std::pair<MatrixXX, VectorX> get_oneLegKinematics_con_cog_foot(const MDT::Pose &base_pose, int legN)
{
    MatrixXX A_output;
    VectorX b_output;
    // base描述world的齐次变换矩阵
    Eigen::Isometry3d T_world_base = base_pose.getT_W_B(); 
    Matrix3 base_world_R = T_world_base.matrix().block(0, 0, 3, 3).transpose();
    Vector3 base_world_p = -base_world_R * T_world_base.matrix().block(0, 3, 3, 1);

    for (int i = 0; i < 6; ++i)
    {
        if(i != legN)
        {
            continue;
        }
        // fixJi坐标系描述base
        Matrix3 Ji_base_R = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[i].matrix().block(0, 0, 3, 3);
        Vector3 Ji_base_p = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[i].matrix().block(0, 3, 3, 1);
        MatrixXX A_foot_Ji;
        VectorX b_foot_Ji;


        // 增加约束,防止腿部工作空间交叉
        switch (i)
        {
            case 0:
                A_foot_Ji = Ab_Ji_foot_forHexMini12.first;
                b_foot_Ji = Ab_Ji_foot_forHexMini12.second;

                A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                b_foot_Ji(b_foot_Ji.rows() - 1) = 0.00;
                break;

            case 1:
                A_foot_Ji = Ab_Ji_foot_forHexMini12.first;
                b_foot_Ji = Ab_Ji_foot_forHexMini12.second;

                A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-0.11;

                A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                b_foot_Ji(b_foot_Ji.rows() - 1) = 0.1;
                break;
            
            case 2:
                A_foot_Ji = Ab_Ji_foot_forHexMini3.first;
                b_foot_Ji = Ab_Ji_foot_forHexMini3.second;
                A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-  0.25;;
            break;

            case 3:
                A_foot_Ji = Ab_Ji_foot_forHexMini45.first;
                b_foot_Ji = Ab_Ji_foot_forHexMini45.second;

                A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                b_foot_Ji(b_foot_Ji.rows() - 1) = 0.00;
            break;

            case 4:
                A_foot_Ji = Ab_Ji_foot_forHexMini45.first;
                b_foot_Ji = Ab_Ji_foot_forHexMini45.second;

                A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-0.11;

                A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                b_foot_Ji(b_foot_Ji.rows() - 1) = 0.1;
            break;

            case 5:
                A_foot_Ji = Ab_Ji_foot_forHexMini6.first;
                b_foot_Ji = Ab_Ji_foot_forHexMini6.second;

                A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-  0.25;;
            break;

            default:
                break;
        }

        A_output.resize(A_foot_Ji.rows(), 3);
        b_output.resize(A_foot_Ji.rows(), 1);

        auto a = A_foot_Ji * Ji_base_R * base_world_R;
        auto b = b_foot_Ji - A_foot_Ji * (Ji_base_R * base_world_p + Ji_base_p);
        A_output = a;
        b_output = b;

    }
    return std::make_pair(A_output, b_output);

}


    std::pair<MatrixXX, VectorX> get_kinematics_con_foot_cog(const MDT::Pose &base_pose, const std::vector<int> &support_leg, const MatrixX3 &contactPoints, const MatrixX3 &contactNormals)
    {
        assert(((int)support_leg.size() == (int)contactPoints.rows()));
        MatrixXX A_output_;
        VectorX b_output_;
        MatrixXX A_foot_Ji;
        VectorX b_foot_Ji;

        // 世界系下的机器人下一步位姿，用于判断下一步fixJi的位置
        Eigen::Isometry3d T_W_B = base_pose.getT_W_B(); 

        // std::cout << "T_W_B.rotation()" << std::endl << T_W_B.rotation() << std::endl;
        for (int i = 0; i < (int)support_leg.size(); i++) //(int)support_leg.size()
        {
            // std::cout << "i:"  << i << std::endl;
            switch (support_leg[i]-1) // 对每条腿设置不同的约束
            {
                case 0:
                    A_foot_Ji = Ab_Ji_foot_forHexMini12.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini12.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.00;
                    A_foot_Ji = -A_foot_Ji; // 设置与Ab_Ji_foot对称
                    break;

                case 1:
                    A_foot_Ji = Ab_Ji_foot_forHexMini12.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini12.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-0.11;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.1;
                    A_foot_Ji = -A_foot_Ji; // 设置与Ab_Ji_foot对称
                    break;
                
                case 2:
                    A_foot_Ji = Ab_Ji_foot_forHexMini3.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini3.second;
                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-  0.25;;
                    A_foot_Ji = -A_foot_Ji; // 设置与Ab_Ji_foot对称
                break;

                case 3:
                    A_foot_Ji = Ab_Ji_foot_forHexMini45.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini45.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.00;
                    A_foot_Ji = -A_foot_Ji; // 设置与Ab_Ji_foot对称
                break;

                case 4:
                    A_foot_Ji = Ab_Ji_foot_forHexMini45.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini45.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-0.11;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << -1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.1; 
                    A_foot_Ji = -A_foot_Ji; // 设置与Ab_Ji_foot对称
                break;

                case 5:
                    A_foot_Ji = Ab_Ji_foot_forHexMini6.first;
                    b_foot_Ji = Ab_Ji_foot_forHexMini6.second;

                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);
                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << 1, 0, 0;
                    b_foot_Ji(b_foot_Ji.rows() - 1) = 0.3-  0.25;;
                    A_foot_Ji = -A_foot_Ji;  // 设置与Ab_Ji_foot对称
                break;

                default:
                    break;
            }

            // 计算foot固定坐标系
            Matrix3 R_W_foot;

            // 旋转矩阵法
            Eigen::Isometry3d T_W_virtualF = Eigen::Isometry3d::Identity();
            T_W_virtualF.rotate(T_W_B.rotation());
            // T_W_virtualF.rotate(Eigen::AngleAxisd( body_fixJi_theta[support_leg[i] - 1] + M_PI/2, Eigen::Vector3d ( 0,0,1) )); // 绕z旋转
            // T_W_virtualF.rotate(Eigen::AngleAxisd( M_PI, Eigen::Vector3d ( 1,0,0) )); // 绕x旋转180度, 让z与B系相反

            R_W_foot = T_W_virtualF.rotation();


            Matrix3 R_foot_W = R_W_foot.transpose();
            Vector3 foot_tmp = R_foot_W * contactPoints.row(i).transpose();  //将接触点转换到足端坐标系下

            // 获取base坐标系下质心到fixJi的偏移
            Vector3 base_fixJi_translation = HexapodParameter::TransMatrix_Body_2_fixJi_forHexMini[support_leg[i] - 1].matrix().block<3, 1>(0, 3);  // 获得每条腿不同的fixJi坐标系下质心到base的偏移

            // 转换为世界坐标系下质心到fixJi的偏移（只有旋转矩阵作用，没有平移！）
            Vector3 W_base_fixJi_translation = T_W_B.matrix().block<3, 3>(0, 0) * base_fixJi_translation;


            // A_output[i].resize(A_foot_Ji.rows(), 3);
            // b_output[i].resize(A_foot_Ji.rows(), 1);

            auto a = A_foot_Ji * R_foot_W;
            auto b = b_foot_Ji - A_foot_Ji * R_foot_W * W_base_fixJi_translation + A_foot_Ji * foot_tmp; 
            // A_output[i] = a;
            // b_output[i] = b;
            // std::cout << "A_output_.shape: (" << A_output_.rows() << "," << A_output_.cols() << ")" << std::endl;

            MatrixXX tempA(A_foot_Ji.rows() + A_output_.rows(), 3);

            MatrixXX tempb(b_foot_Ji.rows() + b_output_.rows(), 1);
            // b_output_.resize(b_output_.rows() + b.rows(), 1);

            if(A_output_.rows() == 0)
            {
                tempA << a;
                tempb << b;
            }
            else
            {
                tempA << A_output_, a;
                tempb << b_output_, b;
            }
            A_output_ = tempA;
            b_output_ = tempb;
        }

        return std::make_pair(A_output_, b_output_);
        
    }





    // 定义不等式约束函数，判断点是否在约束内
    bool isInConvex(const MatrixX3& A, const VectorX& b, const Vector3& pnt) {
        for (int i = 0; i < A.rows(); ++i) {
            if ((A.row(i) * pnt)(0) > b(i)) {
                return false;
            }
        }
        return true;
    }

    // 定义计算射线与凸包交点的函数
    Vector3 findIntersection(const MatrixX3& A, const VectorX& b, const Vector3& pnt, const Vector3& dir) {
        double left = 0.0;
        double right = 1e9; // 可能需要根据实际情况调整右边界

        // 二分法逼近交点
        while (right - left > 0.001) {
            double mid = (left + right) / 2;
            Vector3 intersection = pnt + mid * dir;

            if (isInConvex(A, b, intersection)) {
                left = mid;
            } else {
                right = mid;
            }
        }
        return pnt + left * dir;
    }


    // 计算平面法向量的函数
    Vector3 calculatePlaneNormal(const std::vector<Vector3>& pnts) {
        // 计算质心
        Vector3 centroid(0.0, 0.0, 0.0);
        for (const auto& p : pnts) {
            centroid += p;
        }
        centroid /= pnts.size();

        // 构建坐标矩阵，每个点向质心的向量为新的坐标
        MatrixXX coords(pnts.size(), 3);
        for (int i = 0; i < pnts.size(); ++i) {
            coords.row(i) = pnts[i] - centroid;
        }

        // 使用SVD计算坐标矩阵的最小二乘解
        Eigen::JacobiSVD<MatrixXX> svd(coords, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Vector3 normal = svd.matrixV().col(2);

        return normal.normalized(); // 返回单位法向量
    }




    
}
