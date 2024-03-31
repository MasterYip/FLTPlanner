/*
 * @Date: 2022-12-28 14:33:52
 * @LastEditTime: 2023-04-14 09:36:11
 * @FilePath: /w3_Bretl/src/hit_spider/include/hit_spider/robot_state_transition/solver_Gurobi.hh
 * @Description: 使用Gurobi求解器，求解二次规划问题
 */
#ifndef SOLVER_GUROBI_HH
#define SOLVER_GUROBI_HH

#include <gurobi_c++.h> //Gurobi的头文件
#include "constrains/util.hh"

namespace solvers
{
	// 求解器可能返回的状态
	enum optim_status
	{
		OPTIM_OPTIMAL = 0,
		OPTIM_INFEASIBLE = 1
	};

	static const double MIN_BOUND = -999999.9;
	static const double MAX_BOUND = 999999.9;

	// 定义求解器输出结果
	struct ResultData
	{
		ResultData() : success_(false), cost_(-1.), x(VectorX::Zero(0)) {}

		ResultData(const bool success, const double cost, const VectorX &x) : success_(success), cost_(cost), x(x) {}

		ResultData(const ResultData &other) : success_(other.success_), cost_(other.cost_), x(other.x) {}
		~ResultData() {}

		ResultData &operator=(const ResultData &other)
		{
			success_ = (other.success_);
			cost_ = (other.cost_);
			x = (other.x);
			return *this;
		}
		bool success_; // 求解器是否求解成功
		double cost_;  // 优化的目标代价值
		VectorX x;	   // 决策变量
	};

	// 使用Gurobi求解 min x' h x +  g' x, subject to A*x <= b and D*x = c

	/**
	 * @description: 为Gurobi设置的求解器接口，Eigen矩阵转换为Gurobi所需要的数据类型
	 * @return ResultData：求解器是否求解成功、目标函数值、决策变量值
	 * @param A:线性不等式约束的系数矩阵
	 * @param b:线性不等式约束的右边值
	 * @param D:线性等式约束的系数矩阵
	 * @param d:线性等式约束的右边值
	 * @param Hess:二次目标值构成的矩阵
	 * @param g:线性目标值
	 * @param minBounds:决策变量的下限
	 * @param maxBounds:决策变量的上限
	 *如果想要求最大值就把Hess矩阵和g向量取负号
	 */
	ResultData solve(const MatrixXX &A,
					 const VectorX &b,
					 const MatrixXX &D,
					 const VectorX &d,
					 const MatrixXX &Hess,
					 const VectorX &g,
					 const VectorX &minBounds, const VectorX &maxBounds);

	/**
	 * cpp代码中可以控制是否把gurobi求解的log信息写到控制台
	 * 
	 * @description: Gurobi求解器接口API
	 * @param rows_A:线性不等式约束的个数
	 * @param rows_D:线性等式约束的个数
	 * @param cols:决策变量的维数
	 * @param A:线性不等式约束的系数矩阵
	 * @param b:线性不等式约束的右边值
	 * @param D:线性等式约束的系数矩阵
	 * @param d:线性等式约束的右边值
	 * @param Q:二次目标值
	 * @param c:线性目标值
	 * @param lb:决策变量的下界，不想设置可以传入VectorX::zero(0)，那么默认的下界为-999999.9
	 * @param ub:决策变量的上界，不想设置可以传入VectorX::zero(0)，那么默认的下界为999999.9
	 * @param solution:输出的决策变量的值
	 * @param objvalP:优化出的目标函数值
	 * @param direction = GRB_MINIMIZE :优化的方向，默认是求最小值
	 * @return {*}
	 */
	bool Gurobi_dense_optimize(
		int rows_A,
		int rows_D,
		int cols,
		double *A,
		double *b,
		double *D,
		double *d,
		double *Q,
		double *c,
		double *lb,
		double *ub,
		double *solution,
		double *objvalP,
		int direction = GRB_MINIMIZE);

} /* namespace solvers */

#endif