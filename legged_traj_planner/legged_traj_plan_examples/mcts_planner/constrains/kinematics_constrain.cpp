
#include "constrains/kinematics_constrain.hh"
#include "constrains/my_cdd.hh"
#include "HexMini_IK_interface.hpp"
// #include "hit_spider/hexapod.hh"


namespace Robot_State_Transition
{
    std::pair<MatrixXX, VectorX> getLegKinematicConstraintPolygon_fixJi(int legIndex, bool isReduce, const Parameters &params)
    {
        MatrixXX A_foot_Ji;
        VectorX b_foot_Ji;
        switch (legIndex)
        {
            case 0:
                if(isReduce)
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini12_reduced.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini12_reduced.second;
                }
                else
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini12.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini12.second;
                }


                break;

            case 1:
                if(isReduce)
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini12_reduced.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini12_reduced.second;
                }
                else
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini12.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini12.second;
                }



                break;
            
            case 2:
                if(isReduce)
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini3_reduced.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini3_reduced.second;
                }
                else
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini3.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini3.second;
                }



            break;

            case 3:
                if(isReduce)
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini45_reduced.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini45_reduced.second;
                }
                else
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini45.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini45.second;
                }



            break;

            case 4:
                if(isReduce)
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini45_reduced.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini45_reduced.second;
                }
                else
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini45.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini45.second;
                }


            break;

            case 5:
                if(isReduce)
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini6_reduced.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini6_reduced.second;
                }
                else
                {
                    A_foot_Ji = params.Ab_Ji_foot_forHexMini6.first;
                    b_foot_Ji = params.Ab_Ji_foot_forHexMini6.second;
                }

            
            break;

            default:
                break;
        }
        return std::make_pair(A_foot_Ji, b_foot_Ji);

    }

    // const VectorX b_Ji_foot_forHexMini = initHexMini_b_Ji_foot();
    void get_kinematics_con_cog_foot(const MDT::Pose &base_pose, MatrixXX *A_output, VectorX *b_output, bool isReduce, const Parameters &params)
    {

        // todo:
        // for (int i = 0; i < 6; ++i)
        // {
        //   判断摆动腿在机器人左边系下的足端坐标；然后计算等效碰撞矩形的边界坐标；然后约束后腿的工作空间
        // }

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


            auto result = getLegKinematicConstraintPolygon_fixJi(i, isReduce, params);
            A_foot_Ji = result.first;
            b_foot_Ji = result.second;

            A_output[i].resize(A_foot_Ji.rows(), 3);
            b_output[i].resize(A_foot_Ji.rows(), 1);

            auto a = A_foot_Ji * Ji_base_R * base_world_R;
            auto b = b_foot_Ji - A_foot_Ji * (Ji_base_R * base_world_p + Ji_base_p);
            A_output[i] = a;
            b_output[i] = b;

        }

    }


    bool areCollinear(const Eigen::Vector3d& p, const Eigen::Vector3d& q, const Eigen::Vector3d& r) {
        Eigen::Vector3d pq = q - p;
        Eigen::Vector3d pr = r - p;
        // std::cout << "norm: " << pq.cross(pr).norm() << std::endl;
        return pq.cross(pr).norm() < 1e-4; // 通过向量叉积判断共线性
    }


    double findTheta(const Eigen::Vector3d& edgePoint, const Eigen::Vector3d& AA_FixJi, const Eigen::Vector3d& BB_FixJi, 
                    const Eigen::Isometry3d& T_world_base, const Eigen::Isometry3d& T_Body_2_fixJi, double low, double high) {
        double mid;
        while (high - low > 1e-6) { // 精度
            mid = (low + high) / 2.0;
            Eigen::Isometry3d T_rotate = Eigen::Isometry3d::Identity();
            T_rotate.rotate(Eigen::AngleAxisd(mid, Eigen::Vector3d::UnitZ()));

            Eigen::Vector3d AA_afterRotate_FixJi = T_rotate * AA_FixJi;
            Eigen::Vector3d BB_afterRotate_FixJi = T_rotate * BB_FixJi;

            Eigen::Vector3d AA_afterRotate_world = T_world_base * T_Body_2_fixJi * AA_afterRotate_FixJi;
            Eigen::Vector3d BB_afterRotate_world = T_world_base * T_Body_2_fixJi * BB_afterRotate_FixJi;

            if (areCollinear(edgePoint, AA_afterRotate_world, BB_afterRotate_world)) {
                return mid;
            }

            if ((AA_afterRotate_world - edgePoint).cross(BB_afterRotate_world - edgePoint).z() > 0) {
                high = mid;
            } else {
                low = mid;
            }
            
        }
        return mid;
    }

    float getRotateAngle(const MDT::RobotState &state, int legIndex, Eigen::Vector3d &edgePoint_world, Eigen::Vector3d &AA_, Eigen::Vector3d &BB_)
    {

        double low = -M_PI;
        double high = M_PI; // 0 到 180 度

        // Eigen::Vector3d edgePoint_world{2, 0.3, 0.0};

        // 由于腿构型的不同，设置不同的碰撞点
        Eigen::Vector3d AA_FixJi{0.15, 0.3, 0.0};
        Eigen::Vector3d BB_FixJi{0.15, 0.05, 0.0};
        if(legIndex == 1 || legIndex == 4)
        {
            AA_FixJi.x() = 0.06;
            BB_FixJi.x() = 0.06;
        }



        if(legIndex<3)
        {
            AA_FixJi.y() = -AA_FixJi.y();
            BB_FixJi.y() = -BB_FixJi.y();
        }

        Eigen::Isometry3d T_world_base = state.pose.getT_W_B(); 

        Eigen::Vector3d edgePoint_FixJi = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[legIndex]*T_world_base.inverse() * edgePoint_world;


        double mid;

        while (high - low > 1e-6) { // 精度
            mid = (low + high) / 2.0;
            Eigen::Isometry3d T_rotate = Eigen::Isometry3d::Identity();
            T_rotate.rotate(Eigen::AngleAxisd(mid, Eigen::Vector3d::UnitZ()));

            Eigen::Vector3d AA_afterRotate_FixJi = T_rotate * AA_FixJi;
            Eigen::Vector3d BB_afterRotate_FixJi = T_rotate * BB_FixJi;
            AA_afterRotate_FixJi.z() = 0;  // 只保留平面投影上是平行的
            BB_afterRotate_FixJi.z() = 0;
            edgePoint_FixJi.z() = 0;

            // std::cout << "pointToLineDistance: " << pointToLineDistance(edgePoint_FixJi, AA_afterRotate_FixJi, BB_afterRotate_FixJi) << std::endl;
            AA_ = AA_afterRotate_FixJi;
            BB_ = BB_afterRotate_FixJi;

            if (areCollinear(edgePoint_FixJi, AA_afterRotate_FixJi, BB_afterRotate_FixJi)) {

                return mid;
            }

            if ((AA_afterRotate_FixJi - edgePoint_FixJi).cross(BB_afterRotate_FixJi - edgePoint_FixJi).z() > 0) {
                high = mid;
            }
            else {
                low = mid;
            }
        }

        return mid;
    }



    // 输入是机器人接触状态，输出是摆动腿最大的旋转角度，以防止与支撑腿的碰撞
    std::vector<std::pair<int,float>> getRotateAngleForKinematicConstraints(const MDT::RobotState &state)
    {
        Eigen::Isometry3d T_world_base = state.pose.getT_W_B(); 


        MatrixXX feetPosition_W(6, 3);
        for (int i = 0; i < 6; ++i)
        {
            feetPosition_W.row(i) << state.feetPosition[i].x(), state.feetPosition[i].y(), state.feetPosition[i].z();
        }


        // std::cout << state.gaitToNow << std::endl;
        // std::cout << "support:" << MDT::SUPPORT_FLAG << std::endl; 
        // auto result = wholeBodyIK(state.pose.x, state.pose.y, state.pose.z, state.pose.roll, state.pose.pitch, state.pose.yaw, feetPosition_W);  // need to modify as single leg IK to accelerate it.
        // if (result.first == false)
        // {
        //     std::cout << "IK failed--------!" << std::endl;
        //     exit(0);
        // }
        

        std::vector<std::pair<int,Eigen::Vector3d>> collisionCheckPnts;

        // todo: 判断摆动腿在机器人左边系下的足端坐标；然后计算等效碰撞矩形的边界坐标；然后约束后腿的工作空间
        for (int i = 0; i < 6; ++i)
        {
            if(i == 2 || i == 5) // 后腿不用考虑
            {
                continue;
            }

            // if(state.gaitToNow[i] == MDT::SUPPORT_FLAG)
            if(true)
            {
                // 计算支撑腿足端坐标；
                Eigen::Vector3d foot_FixJi = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[i]*T_world_base.inverse() * state.feetPosition[i];

                float theta = M_PI*3.0f/4.0f;

                auto result_ = leg_IK(state.pose.x, state.pose.y, state.pose.z, state.pose.roll, state.pose.pitch, state.pose.yaw, feetPosition_W, i);
                if (result_.first == false)
                {
                    // std::cout << "IK failed--------!" << std::endl;
                    // getchar();
                }
                else
                {
                    theta = result_.second(0) * (-1.0f);
                }


                Eigen::Vector3d collisionEdgeVertex_A_FixJi{-0.06, 0.3, 0.0};

                if(i<3)
                {
                    theta = -theta;
                    collisionEdgeVertex_A_FixJi.y() = -collisionEdgeVertex_A_FixJi.y();
                }
                

                Eigen::Isometry3d T_rotate = Eigen::Isometry3d::Identity(); // 初始化为单位矩阵
                T_rotate.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));

                // 计算旋转后的碰撞边界点A
                Eigen::Vector3d collisionEdgeVertex_A_afterRotate_FixJi = T_rotate * collisionEdgeVertex_A_FixJi;


                Eigen::Vector3d collisionEdgeVertex_A_afterRotate_world = T_world_base * HexapodParameter::TransMatrix_Body_2_fixJi_forHexMini[i] * collisionEdgeVertex_A_afterRotate_FixJi;

                collisionCheckPnts.push_back(std::make_pair(i,collisionEdgeVertex_A_afterRotate_world));

            }
        }






        std::vector<std::pair<int,float>> results;

        for(auto &pnt: collisionCheckPnts)
        {
            int index = pnt.first + 1; // 支撑腿的后一条腿
            Eigen::Vector3d edgePoint_world = pnt.second;

            Eigen::Vector3d AA_fixJi, BB_fixJi;
            float rotateTheta =  getRotateAngle(state, index, edgePoint_world, AA_fixJi, BB_fixJi);

            results.push_back(std::make_pair(index,rotateTheta));
        }

        return results;
    }


    void get_kinematics_con_cog_foot_selfColision(const MDT::RobotState &state, MatrixXX *A_output, VectorX *b_output, 
        bool isReduce, const Parameters &params)
    {

        auto base_pose = state.pose;
        // base描述world的齐次变换矩阵
        Eigen::Isometry3d T_world_base = base_pose.getT_W_B(); 
        Matrix3 base_world_R = T_world_base.matrix().block(0, 0, 3, 3).transpose();
        Vector3 base_world_p = -base_world_R * T_world_base.matrix().block(0, 3, 3, 1);

        // 获得摆动腿的最大旋转角度
        auto maxAngle = getRotateAngleForKinematicConstraints(state);


        for (int i = 0; i < 6; ++i)
        {
            // fixJi坐标系描述base
            Matrix3 Ji_base_R = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[i].matrix().block(0, 0, 3, 3);
            Vector3 Ji_base_p = HexapodParameter::TransMatrix_FixJi_Body_forHexMini[i].matrix().block(0, 3, 3, 1);
            MatrixXX A_foot_Ji;
            VectorX b_foot_Ji;


            auto result = getLegKinematicConstraintPolygon_fixJi(i, isReduce, params);
            A_foot_Ji = result.first;
            b_foot_Ji = result.second;

            // 添加自碰撞约束
            for(auto &angle: maxAngle)
            {
                if(i == angle.first)
                {
                    float theta = angle.second;
                    A_foot_Ji.conservativeResize(A_foot_Ji.rows() + 1, A_foot_Ji.cols());
                    b_foot_Ji.conservativeResize(b_foot_Ji.rows() + 1);

                    A_foot_Ji.row(A_foot_Ji.rows() - 1) << cos(theta), sin(theta), 0;
                    
                    if(i == 2 && i ==5)
                    {
                        b_foot_Ji(b_foot_Ji.rows() - 1) = -(0.05350);
                    }
                    else
                    {
                        b_foot_Ji(b_foot_Ji.rows() - 1) = (0.05350);
                    }
                    
                }
            }

            A_output[i].resize(A_foot_Ji.rows(), 3);
            b_output[i].resize(A_foot_Ji.rows(), 1);

            auto a = A_foot_Ji * Ji_base_R * base_world_R;
            auto b = b_foot_Ji - A_foot_Ji * (Ji_base_R * base_world_p + Ji_base_p);
            A_output[i] = a;
            b_output[i] = b;

        }





    }



std::pair<MatrixXX, VectorX> get_oneLegKinematics_con_cog_foot(const MDT::Pose &base_pose, int legN, bool isReduce, 
    const Parameters &params)
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

        auto result = getLegKinematicConstraintPolygon_fixJi(i, isReduce, params);
        A_foot_Ji = result.first;
        b_foot_Ji = result.second;

        A_output.resize(A_foot_Ji.rows(), 3);
        b_output.resize(A_foot_Ji.rows(), 1);

        auto a = A_foot_Ji * Ji_base_R * base_world_R;
        auto b = b_foot_Ji - A_foot_Ji * (Ji_base_R * base_world_p + Ji_base_p);
        A_output = a;
        b_output = b;

    }
    return std::make_pair(A_output, b_output);

}


    std::pair<MatrixXX, VectorX> get_kinematics_con_foot_cog(const MDT::Pose &base_pose, const std::vector<int> &support_leg, 
    const MatrixX3 &contactPoints, const MatrixX3 &contactNormals, bool isReduce, const Parameters &params)
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
            int legIndex = support_leg[i] - 1;

            auto result = getLegKinematicConstraintPolygon_fixJi(legIndex, isReduce, params);
            A_foot_Ji = result.first;
            b_foot_Ji = result.second;

            A_foot_Ji = -A_foot_Ji;  // 设置与Ab_Ji_foot对称


            // 计算foot固定坐标系
            Matrix3 R_W_foot;

            // 旋转矩阵法
            Eigen::Isometry3d T_W_virtualF = Eigen::Isometry3d::Identity();
            T_W_virtualF.rotate(T_W_B.rotation());

            R_W_foot = T_W_virtualF.rotation();


            Matrix3 R_foot_W = R_W_foot.transpose();
            Vector3 foot_tmp = R_foot_W * contactPoints.row(i).transpose();  //将接触点转换到足端坐标系下

            // 获取base坐标系下质心到fixJi的偏移
            Vector3 base_fixJi_translation = HexapodParameter::TransMatrix_Body_2_fixJi_forHexMini[support_leg[i] - 1].matrix().block<3, 1>(0, 3);  // 获得每条腿不同的fixJi坐标系下质心到base的偏移

            // 转换为世界坐标系下质心到fixJi的偏移（只有旋转矩阵作用，没有平移！）
            Vector3 W_base_fixJi_translation = T_W_B.matrix().block<3, 3>(0, 0) * base_fixJi_translation;


            auto a = A_foot_Ji * R_foot_W;
            auto b = b_foot_Ji - A_foot_Ji * R_foot_W * W_base_fixJi_translation + A_foot_Ji * foot_tmp; 

            MatrixXX tempA(A_foot_Ji.rows() + A_output_.rows(), 3);

            MatrixXX tempb(b_foot_Ji.rows() + b_output_.rows(), 1);

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
