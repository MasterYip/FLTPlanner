/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-16 21:14:31
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-04-11 10:04:53
 * @FilePath: /w2_cwc/src/hit_spider/include/hit_spider/robot_state_transition/Bretl.hh
 * @Description: Bretl.hh头文件，目前还存在问题，因为是普通指针，需要改成共享指针或者动态指针然后手动delete（在析构函数中解决了）
 */

#ifndef ROBOT_STATE_TRANSITION_BRETL_HH
#define ROBOT_STATE_TRANSITION_BRETL_HH
#include <cmath>
#include "constrains/solve_LP_GLPK.hh"

#define Threshold_Expand 1e-2 // 如果想要更快更简单的集合图形就选择这个
// #define Threshold_Expand 1e-4
#define Threshold_Init 1e-5

namespace Robot_State_Transition
{

    struct Vertex_Planar;  // 二维平面上的顶点
    struct Polygon_Planar; // 二维平面上的多边形

    typedef std::shared_ptr<Vertex_Planar> Vertex_Planar_ptr; // 共享指针

    struct Vertex_Planar
    {
        /**
         * @ : Vertex_Planar的默认构造函数
         * @description:Vertex_Planar的构造函数
         * @return {*}
         */
        Vertex_Planar();

        /**
         * @ : 输入二维点坐标来初始化平面上的点
         * @description:Vertex_Planar的构造函数
         * @param {point_Planar} point_input：输入二维点2×1
         * @return {*}
         */
        Vertex_Planar(point_Planar point_input);

        /**
         * @ : 拓展该顶点
         * @description: 该顶点与它的下一个连接点构成一条edge，沿着edge的垂直方向可以进行拓展
         * @return {*}：返回拓展后的新Vertex顶点的指针
         */
        Vertex_Planar_ptr expand(VectorX &g,
                                 const MatrixXX &A, const VectorX &b,
                                 const MatrixXX &D, const VectorX &d);

        point_Planar point;               // 二维平面上的Vertex顶点坐标
        Vertex_Planar_ptr next = nullptr; // 二维平面Vertex顶点连接的下一个Vertex顶点
        bool expanded = false;            // 该顶点（对应的线）是否拓展过
    };

    struct Polygon_Planar
    {
        // 默认构造函数
        Polygon_Planar();

        /**
         * @ : 用三个顶点来初始化平面多边形
         * @description: Polygon_Planar的构造函数
         * @param {Vertex_Planar} *v1：顶点1，连接顶点2
         * @param {Vertex_Planar} *v2：顶点2，连接顶点3
         * @param {Vertex_Planar} *v3：顶点3，连接顶点1
         * @return {*}
         */
        Polygon_Planar(const Vertex_Planar &v1, const Vertex_Planar &v2, const Vertex_Planar &v3);

        /**
         * @ : 多边形所有顶点是否全部拓展完
         * @description: 所有顶点全部多拓展完有两种情况：1：线性多边形和和高维凸包的投影完全重合；2：线性多边形边线和高维凸包投影边线很接近，误差满足一定范围
         * @return {*}
         */
        bool all_expanded() const;

        /**
         * @ :根据约束条件循环拓展多边形直至到达最大迭代次数
         * @description:
         * @param {VectorX} &g：优化目标值，输入的只是一个壳子
         * @param {MatrixXX} &A：不等式约束的约束矩阵
         * @param {VectorX} &b：不等式约束的右边值
         * @param {MatrixXX} &D：等式约束的约束矩阵
         * @param {VectorX} &d：等式约束的右边值
         * @param {int} &max_iter：迭代的最大次数
         * @return {*}
         */
        void iter_expand(VectorX g,
                         const MatrixXX &A, const VectorX &b,
                         const MatrixXX &D, const VectorX &d,
                         const int &max_iter);

        /**
         * @ : 输出间距大于min_dist的顶点集合
         * @description:
         * @param {double} min_dist：最小的顶点间距
         * @return {*}：间距大于min_dist的顶点集合
         */
        MatrixXX export_vertices(double min_dist = 1e-2) const;

        std::vector<Vertex_Planar_ptr> vertices; // 构成平面多边形的Vertex顶点指针集合
    };

    /**
     * @ : 朝着一个方向进行优化，获取该方向上的边界点
     * @description:
     * @param {point_Planar} &vdir：二维平面上优化的方向
     * @param {VectorX} &g：优化目标值，输入的只是一个壳子
     * @param {MatrixXX} &A：不等式约束的约束矩阵
     * @param {VectorX} &b：不等式约束的右边值
     * @param {MatrixXX} &D：等式约束的约束矩阵
     * @param {VectorX} &d：等式约束的右边值
     * @param {point_Planar} &result_opt：如果优化成功会返回平面上新扩展的点
     * @return {*}：求解是否成功
     */
    bool optimize_direction(const point_Planar &vdir, VectorX &g,
                            const MatrixXX &A, const VectorX &b,
                            const MatrixXX &D, const VectorX &d,
                            point_Planar &result_opt);

    /**
     * @ :朝着theta定义的方向进行拓展
     * @description:
     * @param {double} theta：与x轴的夹角定义拓展方向
     * @param {VectorX} &g：优化目标值，输入的只是一个壳子
     * @param {MatrixXX} &A：不等式约束的约束矩阵
     * @param {VectorX} &b：不等式约束的右边值
     * @param {MatrixXX} &D：等式约束的约束矩阵
     * @param {VectorX} &d：等式约束的右边值
     * @param {point_Planar} &result_opt：如果优化成功会返回平面上新扩展的点
     * @return {*}：求解是否成功
     */
    bool optimize_angle(const double theta, VectorX &g,
                        const MatrixXX &A, const VectorX &b,
                        const MatrixXX &D, const VectorX &d,
                        point_Planar &result_opt);

    /**
     * @ :
     * @description:
     * @param {VectorX} &g：优化目标值，输入的只是一个壳子
     * @param {MatrixXX} &A：不等式约束的约束矩阵
     * @param {VectorX} &b：不等式约束的右边值
     * @param {MatrixXX} &D：等式约束的约束矩阵
     * @param {VectorX} &d：等式约束的右边值
     * @param {bool} &if_success：是否求解成功
     * @param {double} *init_angle：初始拓展角度（决定初始三个点）
     * @param {int} max_iter：最大迭代次数（初始三个点算入其中）
     * @return {Polygon_Planar}：
     */
    Polygon_Planar compute_polygon(VectorX &g,
                                   const MatrixXX &A, const VectorX &b,
                                   const MatrixXX &D, const VectorX &d,
                                   bool &if_success,
                                   double *init_angle,
                                   int max_iter);

    /**
     * @ : 高维凸包x投影为y=Ex+f，x定义为:Ax<=b;Dx=d;
     * @description:
     * @param {MatrixXX} &E：y = Ex + f
     * @param {VectorX} &f：
     * @param {MatrixXX} &A：Ax <= b
     * @param {VectorX} &b：
     * @param {MatrixXX} &D：Dx = d
     * @param {VectorX} &d：
     * @param {MatrixXX} &output_Vertices：投影多边形的顶点集合
     * @param {double} *init_angle：初始拓展方向
     * @param {int} max_iter：最大拓展次数
     * @param {double} max_radius：y的范围
     * @return {*}：是否成功
     */
    bool project_polytope_bretl(const MatrixXX &E, const VectorX &f,
                                const MatrixXX &A, const VectorX &b,
                                const MatrixXX &D, const VectorX &d,
                                MatrixXX &output_Vertices,
                                double *init_angle = nullptr,
                                int max_iter = 1000, const double max_radius = 1e5);

    /**
     * @ : 计算由CWC决定的质心平面XY可行域，顺时针排序的点！每一列是一个点！
     * @description: 用户接口
     * @param {MatrixX3} &contactPoints：接触点位置，每一行是一个位置
     * @param {MatrixX3} &contactNormals：接触点法向量，每一行是一个法向量
     * @param {double} frictionCoefficient：摩擦系数
     * @return {*}：返回的点是按照顺时针排序的！
     */
    bool comput_friction_region(const MatrixX3 &contactPoints, const MatrixX3 &contactNormals_input, const double frictionCoefficient, const double m, MatrixXX &output_Vertices);

    std::pair<MatrixXX, VectorX> Bretl_Vettices_to_face(const MatrixXX &Vertices);

    bool comput_friction_region_considerMaxF(const MatrixX3 &contactPoints, const MatrixX3 &contactNormals_input, VectorX &contact_maxmumF, const double frictionCoefficient_input, const double m, MatrixXX &output_Vertices);
}



#endif