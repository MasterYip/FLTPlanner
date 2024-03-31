#ifndef UTIL_HH
#define UTIL_HH
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>    //STL容器(可以尝试使用list保存备选节点)
#include <algorithm> //STL算法
#include <utility>   //std::pair
#include <memory>    //shared智能指针动态内存分配


//------数据类型的定义------
typedef Eigen::Matrix<double, 2, 1> point_Planar;          // 二维平面上的点
typedef Eigen::Matrix<double, 3, 1> Vector3;               // 三维点
typedef Eigen::Matrix<double, 6, 1> Vector6;               // 六维状态
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorX;  // 动态维数向量
typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VectorXi;
typedef Eigen::Matrix<double, 3, 3> Matrix3;               // 3×3矩阵
typedef Eigen::Matrix<double, 6, 3> Matrix63;              // 6×3矩阵，六条腿的关节角度
typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatrixX3; // 三维向量按行排列的，可用于表示接触点、法向量序列（不知道有多少接触点）、雅克比矩阵序列
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;

Matrix3 crossMatrix(const Vector3 &x); // 求三维向量对应的反对称矩阵


#endif