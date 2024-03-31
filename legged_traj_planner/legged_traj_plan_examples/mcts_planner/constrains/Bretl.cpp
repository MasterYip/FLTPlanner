/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-16 21:11:07
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-04-13 09:54:38
 * @FilePath: /w3_Bretl/src/hit_spider/src/robot_state_transition/Bretl.cpp
 * @Description: pypoman.Bretl.py的复现
 */
#include "constrains/Bretl.hh"
// #define PI 3.14159265358979323846

namespace Robot_State_Transition
{
    namespace
    {
        void my_round(double &num)
        {
            num = num + 0.0;

            // double temp = num * (double)100;
            // temp = round(temp);
            // num = temp / (double)100;
        }
    }

    // Vertex_Planar部分
    Vertex_Planar::Vertex_Planar()
    {
        this->point.setZero();
        this->next = nullptr;
        this->expanded = false;
    }

    Vertex_Planar::Vertex_Planar(point_Planar point_input)
    {
        this->point = point_input;
        this->next = nullptr;
        this->expanded = false;
    }

    Vertex_Planar_ptr Vertex_Planar::expand(VectorX &g,
                                            const MatrixXX &A, const VectorX &b,
                                            const MatrixXX &D, const VectorX &d)
    {
        // 优化方向
        //  v1 = this
        Vertex_Planar_ptr v2 = this->next;
        point_Planar direction;

        direction(0) = (v2->point)(1) - (this->point)(1); // 拓展方向，垂直于边线，指向外部
        direction(1) = (this->point)(0) - (v2->point)(0);
        direction.normalize();

        // 进行优化
        point_Planar point_boundary;
        bool flag = optimize_direction(direction, g, A, b, D, d, point_boundary);
        if (!flag)
        {
            this->expanded = true;
            return nullptr;
        }

        // 是否添加添加新得到的边界点
        point_Planar p1 = point_boundary - this->point;
        point_Planar p2 = v2->point - this->point;

        double dis_expand = abs(p1(0) * p2(1) - p1(1) * p2(0));
        if (dis_expand < Threshold_Expand)
        {
            this->expanded = true;
            return nullptr;
        }
        else
        {
            Vertex_Planar_ptr vnew = std::make_shared<Vertex_Planar>(point_boundary);
            // vnew连接原来的两个点
            vnew->next = this->next;
            this->next = vnew;
            this->expanded = false;
            return vnew;
        }
    }

    // Polygon_Planar部分
    Polygon_Planar::Polygon_Planar()
    {
        this->vertices.clear();
    }

    Polygon_Planar::Polygon_Planar(const Vertex_Planar &v1, const Vertex_Planar &v2, const Vertex_Planar &v3)
    {
        Vertex_Planar_ptr pv1 = std::make_shared<Vertex_Planar>(v1);
        Vertex_Planar_ptr pv2 = std::make_shared<Vertex_Planar>(v2);
        Vertex_Planar_ptr pv3 = std::make_shared<Vertex_Planar>(v3);

        pv1->next = pv2;
        pv2->next = pv3;
        pv3->next = pv1;

        this->vertices.push_back(pv1);
        this->vertices.push_back(pv2);
        this->vertices.push_back(pv3);
    }

    bool Polygon_Planar::all_expanded() const
    {
        for (auto iter = vertices.begin(); iter != vertices.end(); ++iter)
        {
            if (!(*iter)->expanded)
            {
                return false;
            }
        }
        return true;
    }

    void Polygon_Planar::iter_expand(VectorX g,
                                     const MatrixXX &A, const VectorX &b,
                                     const MatrixXX &D, const VectorX &d,
                                     const int &max_iter)
    {
        int nb_iter = 0;
        Vertex_Planar_ptr v = this->vertices.at(0);
        while (!(this->all_expanded()) && nb_iter < max_iter)
        {
            if (v->expanded)
            {
                v = v->next;
                continue;
            }
            Vertex_Planar_ptr vnew = v->expand(g, A, b, D, d);
            if (vnew == nullptr)
            {
                continue;
            }
            this->vertices.push_back(vnew);
            ++nb_iter;
        }
    }

    MatrixXX Polygon_Planar::export_vertices(double min_dist) const
    {
        std::vector<Vertex_Planar_ptr> newVertices;
        newVertices.push_back(*(this->vertices.begin())); // 加入第一个点
        Vertex_Planar_ptr v_export = *(this->vertices.begin());

        do
        {
            v_export = v_export->next;
            Vertex_Planar_ptr vlast = *(newVertices.end() - 1);
            if ((v_export->point - vlast->point).norm() > min_dist)
            {
                newVertices.push_back(v_export);
            }
        } while (v_export->next != *(this->vertices.begin()));

        // 转换为矩阵
        MatrixXX output_matrix;
        output_matrix.resize(2, newVertices.size());
        int num_col = 0;
        for (auto iter = newVertices.begin(); iter != newVertices.end(); ++iter, ++num_col)
        {
            output_matrix.col(num_col) = (*iter)->point;
        }

        for (int i = 0; i < output_matrix.rows(); ++i)
        {
            for (int j = 0; j < output_matrix.cols(); ++j)
            {
                my_round(output_matrix(i, j));
            }
        }

        return output_matrix;
    }

    bool optimize_direction(const point_Planar &vdir, VectorX &g,
                            const MatrixXX &A, const VectorX &b,
                            const MatrixXX &D, const VectorX &d,
                            point_Planar &result_opt)
    {
        int g_rows = g.rows();
        g(g_rows - 2) = -vdir(0); // x
        g(g_rows - 1) = -vdir(1); // y
        VectorX vec_opt;
        bool flag = solve_LP_Bretl_GLPK(A, b, D, d, g, vec_opt);
        if (flag)
        {
            result_opt << vec_opt(g_rows - 2), vec_opt(g_rows - 1);
        }

        return flag;
    }

    bool optimize_angle(const double theta, VectorX &g,
                        const MatrixXX &A, const VectorX &b,
                        const MatrixXX &D, const VectorX &d,
                        point_Planar &result_opt)
    {
        point_Planar vdir;
        vdir(0) = cos(theta);
        vdir(1) = sin(theta);

        // debug
        // std::cout << "优化的方向为:\n"
        //           << vdir.transpose() << std::endl;

        bool flag = optimize_direction(vdir, g, A, b, D, d, result_opt);
        return flag;
    }

    Polygon_Planar compute_polygon(VectorX &g,
                                   const MatrixXX &A, const VectorX &b,
                                   const MatrixXX &D, const VectorX &d,
                                   bool &if_success,
                                   double *init_angle, int max_iter)
    {
        if_success = false;

        bool flag_angle = false;
        if (init_angle == nullptr)
        {
            flag_angle = true;
            init_angle = new double((double)rand() / RAND_MAX * M_PI);
        }
        double theta1, theta2, theta3, theta23;
        theta1 = theta23 = *init_angle;
        double step_angle = (double)2 / 3 * M_PI;

        MatrixXX init_vertices;
        init_vertices.resize(2, 3); // 三个初始点
        int init_num = 0, init_iter_num = 0;
        point_Planar point;


        // 第一个初始点，一定能找到
        while (true)
        {
            bool flag = optimize_angle(theta1, g, A, b, D, d, point);
            if (flag)
            {
                init_vertices.col(init_num++) = point;
                break;
            }
            else
            {
                return Polygon_Planar();
            }
        }



        // debug
        // std::cout << "第一个顶点为: \n"
        //           << init_vertices << std::endl;

        // 另外两个初始点
        while (init_num < 3 && init_iter_num < 100)
        {
            theta23 += step_angle;
            if (theta23 >= 2 * M_PI)
            {
                step_angle *= 0.25 + 0.5 * (double)random() / RAND_MAX;
                theta23 += step_angle - 2 * M_PI;
            }

            bool flag = optimize_angle(theta23, g, A, b, D, d, point);
            if (flag)
            {

                ++init_iter_num;
                // debug
                // std::cout << "优化出的点为:\n"
                //           << point << std::endl;

                // 防止初始点之间距离过小
                if (init_num == 1)
                {
                    if ((point - init_vertices.col(0)).norm() < Threshold_Init)
                    {
                        continue;
                    }
                    init_vertices.col(init_num++) = point;
                    theta2 = theta23;

                    // debug
                    // std::cout << "现在初始顶点集合为:\n"
                    //           << init_vertices << std::endl;
                }
                else if (init_num == 2)
                {
                    if ((point - init_vertices.col(0)).norm() < Threshold_Init || (point - init_vertices.col(1)).norm() < Threshold_Init)
                    {
                        continue;
                    }
                    init_vertices.col(init_num++) = point;
                    theta3 = theta23;

                    // debug
                    // std::cout << "现在初始顶点集合为:\n"
                    //           << init_vertices << std::endl;
                }
            }
        }
        max_iter -= init_iter_num;

        // 这个可不是debug，千万别删除啊!
        // assert(init_num == 3); // 一定需要有三个初始点
        if (init_num != 3)
        {
            Polygon_Planar polygon;
            if_success = false;
            return polygon;
        }

        if (theta1 > M_PI)
        {
            theta1 -= 2 * M_PI;
        }
        if (theta2 > M_PI)
        {
            theta2 -= 2 * M_PI;
        }
        if (theta3 > M_PI)
        {
            theta3 -= 2 * M_PI;
        }

        assert((theta1 != theta2) && (theta1 != theta3) && (theta2 != theta3));
        Vertex_Planar v1, v2, v3;
        // 根据初始点的三个角度进行排序赋值，逆时针连接三个点
        if (theta1 > theta2)
        {
            // theta1 > theta2

            if (theta1 > theta3)
            {
                // theta1最大
                if (theta2 > theta3)
                {
                    // theta1 > theta2 > theta3
                    v1 = Vertex_Planar(init_vertices.col(0));
                    v2 = Vertex_Planar(init_vertices.col(2));
                    v3 = Vertex_Planar(init_vertices.col(1));
                }
                else
                {
                    // theta1 > theta3 > theta2
                    v1 = Vertex_Planar(init_vertices.col(0));
                    v2 = Vertex_Planar(init_vertices.col(1));
                    v3 = Vertex_Planar(init_vertices.col(2));
                }
            }
            else
            {
                // theta3最大
                //  theta3 > theta1 > theta2
                v1 = Vertex_Planar(init_vertices.col(2));
                v2 = Vertex_Planar(init_vertices.col(1));
                v3 = Vertex_Planar(init_vertices.col(0));
            }
        }
        else
        {
            // theta2 > theta1

            if (theta2 > theta3)
            {
                // theta2最大
                if (theta1 > theta3)
                {
                    // theta2 > theta1 > theta3
                    v1 = Vertex_Planar(init_vertices.col(1));
                    v2 = Vertex_Planar(init_vertices.col(2));
                    v3 = Vertex_Planar(init_vertices.col(0));
                }
                else
                {
                    // theta2 > theta3 > theta1
                    v1 = Vertex_Planar(init_vertices.col(1));
                    v2 = Vertex_Planar(init_vertices.col(0));
                    v3 = Vertex_Planar(init_vertices.col(2));
                }
            }
            else
            {
                // theta3 > theta2 > theta1
                v1 = Vertex_Planar(init_vertices.col(2));
                v2 = Vertex_Planar(init_vertices.col(0));
                v3 = Vertex_Planar(init_vertices.col(1));
            }
        }

        // debug
        // std::cout << "三个初始点排序后为:\n"
        //           << v1.point.transpose() << '\n'
        //           << v2.point.transpose() << '\n'
        //           << v3.point.transpose()
        //           << std::endl;

        Polygon_Planar polygon(v1, v2, v3);
        polygon.iter_expand(g, A, b, D, d, max_iter);

        // debug
        // std::cout << "计算出的点为:\n"
        //           << std::endl;
        // Vertex_Planar_ptr v_print = polygon.vertices[0];
        // do
        // {
        //     std::cout << v_print->point.transpose() << std::endl;
        //     v_print = v_print->next;
        // } while (v_print != polygon.vertices[0]);

        if (flag_angle)
        {
            delete init_angle;
        }

        if_success = true;
        return polygon;
    }

    //------
    bool project_polytope_bretl(const MatrixXX &E, const VectorX &f,
                                const MatrixXX &A, const VectorX &b,
                                const MatrixXX &D, const VectorX &d,
                                MatrixXX &output_Vertices,
                                double *init_angle,
                                int max_iter, const double max_radius)
    {
        output_Vertices.resize(0, 0);

        // 投影到二维平面上
        assert(E.rows() == 2 && f.rows() == 2);

        // 加入范围不等式约束 A_extand拓展
        // Inequality constraints: A_ext * [ x  u  v ] <= b_ext iff
        // (1) A * x <= b and (2) |u|, |v| <= max_radius
        MatrixXX A_ext;
        const int rows_A = A.rows(); // 需要是const才能在下面使用
        const int cols_A = A.cols();

        A_ext.resize(rows_A + 4, cols_A + 2);
        A_ext.setZero();
        A_ext.block(0, 0, rows_A, cols_A) = A;
        A_ext(rows_A, cols_A) = 1;
        A_ext(rows_A + 1, cols_A) = -1;
        A_ext(rows_A + 2, cols_A + 1) = 1;
        A_ext(rows_A + 3, cols_A + 1) = -1;

        VectorX b_ext;
        b_ext.resize(rows_A + 4);
        b_ext.setZero();
        b_ext.block(0, 0, rows_A, 1) = b;
        b_ext.block(rows_A, 0, 4, 1).array() = max_radius;

        // 加入等式约束
        // Equality constraints: C_ext * [ x  u  v ] == d_ext iff
        // (1) C * x == d and (2) [ u  v ] == E * x + f
        MatrixXX D_ext;
        const int rows_D = D.rows();
        const int cols_D = D.cols();
        D_ext.resize(rows_D + 2, cols_D + 2);
        D_ext.setZero();
        D_ext.block(0, 0, rows_D, cols_D) = D;
        D_ext.block(rows_D, 0, 2, cols_D) = E;
        D_ext.block(rows_D, cols_D, 2, 2) = -1 * Eigen::Matrix<double, 2, 2>::Identity();

        VectorX d_ext;
        d_ext.resize(rows_D + 2, 1);
        d_ext.setZero();
        d_ext.block(0, 0, rows_D, 1) = d;
        d_ext.block(rows_D, 0, 2, 1) = -1 * f;

        VectorX g;
        g.resize(cols_A + 2, 1);
        g.setZero();

        // // debug
        // std::cout << "g = \n"
        //           << g << std::endl;
        // std::cout << "A_ext = \n"
        //           << A_ext << std::endl;
        // std::cout << "b_ext = \n"
        //           << b_ext << std::endl;
        // std::cout << "D_ext = \n"
        //           << D_ext << std::endl;
        // std::cout << "d_ext = \n"
        //           << d_ext << std::endl;

        bool is_success = false;
        Polygon_Planar polygon = compute_polygon(g, A_ext, b_ext, D_ext, d_ext, is_success, init_angle, max_iter);

        if (is_success)
        {
            output_Vertices = (polygon.export_vertices()).transpose();
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief 计算稳定多边形
     * @param contactPoints 接触点
     * @param contactNormals_input 接触点法向量
     * @param frictionCoefficient 摩擦系数
     * @param m 质量
     * @param output_Vertices 输出的稳定多边形顶点
     * @return 是否计算成功 
    */

    bool comput_friction_region(const MatrixX3 &contactPoints, const MatrixX3 &contactNormals_input, const double frictionCoefficient, const double m, MatrixXX &output_Vertices)
    {
        double g = 9.81;
        // 接触点数量、法向量数量需要保持一致
        assert(contactPoints.rows() == contactNormals_input.rows());
        MatrixX3 contactNormals = contactNormals_input.rowwise().normalized(); // 支撑点法向量归一化

        const int contact_Num = contactPoints.rows();
        const int cone_edge_number = 4;
        const double delta_theta = 2 * (double)M_PI / cone_edge_number; // 线性化摩擦锥需要的变化角度，默认是pi/2

        // CWC计算
        //------一个接触点一个接触点进行计算------
        //  变量定义参考质心动力学方程
        MatrixXX A1(6, 3 * contact_Num); // 质心动力学约束矩阵
        A1.setZero();
        VectorX u(6, 1);
        u.setZero();
        u(2, 0) = m * g; // 重力

        MatrixXX B(4 * contact_Num, 3 * contact_Num); // 线性摩擦锥约束矩阵
        B.setZero();

        MatrixXX A1_temp(6, 3); // A = [I3;Skew_symmetric_Matrix(接触点p)]
        A1_temp.setZero();
        A1_temp.block<3, 3>(0, 0).setIdentity();

        MatrixXX B_temp(3, 4);

        for (int i = 0; i < contact_Num; ++i)
        {
            Vector3 N = contactNormals.row(i);
            A1_temp.block<3, 3>(3, 0) = crossMatrix(contactPoints.row(i).transpose());

            // 计算与法向量垂直的切向量
            Vector3 T1, T2;
            T1 = N.cross(Vector3::UnitY());
            if (T1.norm() < 1e-5)
            {
                T1 = N.cross(Vector3::UnitX());
            }
            T2 = N.transpose().cross(T1);

            T1.normalize();
            T2.normalize();

            // debug
            // std::cout << "T1 = \n"
            //           << T1 << std::endl;
            // std::cout << "T2 = \n"
            //           << T2 << std::endl;

            // 四棱锥的线性约束
            B_temp.col(0) = T1 - frictionCoefficient * N;
            B_temp.col(1) = T2 - frictionCoefficient * N;
            B_temp.col(2) = -1 * T1 - frictionCoefficient * N;
            B_temp.col(3) = -1 * T2 - frictionCoefficient * N;

            A1.block(0, 3 * i, 6, 3) = A1_temp;
            B.block(4 * i, 3 * i, 4, 3) = B_temp.transpose();
        }

        // c_xy = Ex + f
        MatrixXX E(2, 3 * contact_Num);
        VectorX f(2);
        E.row(0) = A1.row(4) / (-1 * m * g); // x坐标
        E.row(1) = A1.row(3) / (m * g);      // y坐标
        f.setZero();

        // Ax == b
        MatrixXX A(4, 3 * contact_Num);
        VectorX b(4);
        A.block(0, 0, 3, 3 * contact_Num) = A1.block(0, 0, 3, 3 * contact_Num);
        A.block(3, 0, 1, 3 * contact_Num) = A1.block(5, 0, 1, 3 * contact_Num);
        b.setZero();
        b(2) = m * g;

        // Dx <= d
        // MatrixXX D = B
        VectorX d(4 * contact_Num);
        d.setZero();

        // // debug
        // std::cout << "E = \n"
        //           << E << std::endl;
        // std::cout << "f = \n"
        //           << f << std::endl;
        // std::cout << "A = \n"
        //           << A << std::endl;
        // std::cout << "b = \n"
        //           << b << std::endl;
        // std::cout << "D = \n"
        //           << B << std::endl;
        // std::cout << "d = \n"
        //           << d << std::endl;

        // srand(10); // 确保初始init_angle的角度相同，可以复现实验

        return project_polytope_bretl(E, f, B, d, A, b, output_Vertices);
    }


    /**
     * @brief 计算稳定多边形
     * @param contactPoints 接触点
     * @param contactNormals_input 接触点法向量
     * @param contact_maxmumF 接触点法向最大力
     * @param frictionCoefficient 摩擦系数
     * @param m 质量
     * @param output_Vertices 输出的稳定多边形顶点
     * @return 是否计算成功 
    */
    bool comput_friction_region_considerMaxF(const MatrixX3 &contactPoints, const MatrixX3 &contactNormals_input, VectorX &contact_maxmumF, const double frictionCoefficient_input, const double m, MatrixXX &output_Vertices)
    {
        // MatrixX3 contact_maxmumF(3, 3);
        // VectorX contactF(3);
        // contactF << 1000, 1000, 1000;
        // contact_maxmumF << 0, 0, 10000,
        //     0, 0, 500,
        //     0, 0, 500;

        
        // double frictionCoefficient = frictionCoefficient_input / sqrt(2.0);
        double frictionCoefficient = frictionCoefficient_input;
        double g = 9.81;
        // 接触点数量、法向量数量需要保持一致
        assert(contactPoints.rows() == contactNormals_input.rows());
        MatrixX3 contactNormals = contactNormals_input.rowwise().normalized(); // 支撑点法向量归一化

        const int contact_Num = contactPoints.rows();
        const int cone_edge_number = 4;
        const double delta_theta = 2 * (double)M_PI / cone_edge_number; // 线性化摩擦锥需要的变化角度，默认是pi/2

        // CWC计算
        //------一个接触点一个接触点进行计算------
        //  变量定义参考质心动力学方程
        MatrixXX A1(6, 3 * contact_Num); // 质心动力学约束矩阵
        A1.setZero();
        VectorX u(6, 1);
        u.setZero();
        u(2, 0) = m * g; // 重力

        MatrixXX B(5 * contact_Num, 3 * contact_Num); // 线性摩擦锥约束矩阵
        B.setZero();

        MatrixXX A1_temp(6, 3); // A = [I3;Skew_symmetric_Matrix(接触点p)]
        A1_temp.setZero();
        A1_temp.block<3, 3>(0, 0).setIdentity();

        MatrixXX B_temp(3, 5);

        for (int i = 0; i < contact_Num; ++i)
        {
            Vector3 N = contactNormals.row(i);
            A1_temp.block<3, 3>(3, 0) = crossMatrix(contactPoints.row(i).transpose());  // A1矩阵的下三行, skewsymmetric matrix formed by the contact point position vector

            // 计算与法向量垂直的切向量
            Vector3 T1, T2;
            T1 = N.cross(Vector3::UnitY());
            if (T1.norm() < 1e-5)
            {
                T1 = N.cross(Vector3::UnitX());
            }
            T2 = N.transpose().cross(T1);

            T1.normalize();
            T2.normalize();

            // debug
            // std::cout << "T1 = \n"
            //           << T1 << std::endl;
            // std::cout << "T2 = \n"
            //           << T2 << std::endl;

            // 四棱锥的线性约束
            B_temp.col(0) = T1 + T2 - frictionCoefficient * N;
            B_temp.col(1) = T1 - T2 - frictionCoefficient * N;
            B_temp.col(2) = -1 * T1 + T2 - frictionCoefficient * N;
            B_temp.col(3) = -1 * T1 - T2 - frictionCoefficient * N;


            // 在这里加新的不等式约束
            B_temp.col(4) = N;
            
            // B_temp.col(0) = T1 - frictionCoefficient * N;
            // B_temp.col(1) = T2 - frictionCoefficient * N;
            // B_temp.col(2) = -1 * T1 - frictionCoefficient * N;
            // B_temp.col(3) = -1 * T2 - frictionCoefficient * N;

            A1.block(0, 3 * i, 6, 3) = A1_temp;
            B.block(5 * i, 3 * i, 5, 3) = B_temp.transpose();
        }

        // c_xy = Ex + f
        MatrixXX E(2, 3 * contact_Num);
        VectorX f(2);
        E.row(0) = A1.row(4) / (-1 * m * g); // x坐标
        E.row(1) = A1.row(3) / (m * g);      // y坐标
        f.setZero();

        // Ax == b
        MatrixXX A(4, 3 * contact_Num);
        VectorX b(4);
        A.block(0, 0, 3, 3 * contact_Num) = A1.block(0, 0, 3, 3 * contact_Num);
        A.block(3, 0, 1, 3 * contact_Num) = A1.block(5, 0, 1, 3 * contact_Num);
        b.setZero();
        b(2) = m * g;

        // Dx <= d
        // MatrixXX D = B
        VectorX d(5 * contact_Num);
        d.setZero();
        for (int i = 0; i < contact_Num; ++i)
        {
            d(5 * i+4) = contact_maxmumF(i);
        }

        // // debug
        // std::cout << "E = \n"
        //           << E << std::endl;
        // std::cout << "f = \n"
        //           << f << std::endl;
        // std::cout << "A = \n"
        //           << A << std::endl;
        // std::cout << "b = \n"
        //           << b << std::endl;
        // std::cout << "D = \n"
        //           << B << std::endl;
        // std::cout << "d = \n"
        //           << d << std::endl;

        // srand(10); // 确保初始init_angle的角度相同，可以复现实验

        // B, d 不等式约束系数
        // A, b 等式约束系数
        return project_polytope_bretl(E, f, B, d, A, b, output_Vertices, nullptr);
    }


    std::pair<MatrixXX, VectorX> Bretl_Vettices_to_face(const MatrixXX &Vertices)
    {
        const int rows_num = Vertices.rows();

        MatrixXX A(rows_num, 2);
        VectorX B(rows_num);

        const point_Planar COG((Vertices.col(0).sum()) / (double)rows_num, (Vertices.col(1).sum()) / (double)rows_num);
        double a, b, c, d;
        for (int i = 0; i < rows_num - 1; ++i)
        {
            a = Vertices(i + 1, 1) - Vertices(i, 1);                                       // y2 - y1
            b = Vertices(i, 0) - Vertices(i + 1, 0);                                       // x1 - x2
            c = Vertices(i + 1, 0) * Vertices(i, 1) - Vertices(i, 0) * Vertices(i + 1, 1); // x2*y1 - x1*y2
            d = a * COG(0) + b * COG(1) + c;

            // 不是debug，不能删除
            assert(d != 0); // 凸包，中心不可能在棱边上

            if (d > 0)
            {
                A(i, 0) = -a;
                A(i, 1) = -b;
                B(i) = c;
            }
            else
            {
                A(i, 0) = a;
                A(i, 1) = b;
                B(i) = -c;
            }
        }
        a = Vertices(0, 1) - Vertices(rows_num - 1, 1);                                              // y2 - y1
        b = Vertices(rows_num - 1, 0) - Vertices(0, 0);                                              // x1 - x2
        c = Vertices(0, 0) * Vertices(rows_num - 1, 1) - Vertices(rows_num - 1, 0) * Vertices(0, 1); // x2*y1 - x1*y2
        d = a * COG(0) + b * COG(1) + c;

        // 不是debug，不能删除
        assert(d != 0); // 凸包，中心不可能在棱边上

        if (d > 0)
        {
            A(rows_num - 1, 0) = -a;
            A(rows_num - 1, 1) = -b;
            B(rows_num - 1) = c;
        }
        else
        {
            A(rows_num - 1, 0) = a;
            A(rows_num - 1, 1) = b;
            B(rows_num - 1) = -c;
        }

        return std::make_pair(A, B);
    }   


}