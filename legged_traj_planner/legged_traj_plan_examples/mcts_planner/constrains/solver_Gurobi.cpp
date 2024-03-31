/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-14 16:23:36
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-04-14 09:45:06
 * @FilePath: /w3_Bretl/src/hit_spider/src/robot_state_transition/solver_Gurobi.cpp
 * @Description: 
 */
/*
 * @Date: 2022-12-28 14:55:45
 * @LastEditTime: 2023-02-22 16:49:06
 * @FilePath: /test_include/src/src/CROC/solver_Gurobi.cpp
 * @Description: Gurobi求解接口的实现
 */
#include "constrains/solver_Gurobi.hh"
namespace solvers
{

	ResultData solve(const MatrixXX &Matrix_A, const VectorX &Vector_b,
					 const MatrixXX &Matrix_D, const VectorX &Vector_d,
					 const MatrixXX &Matrix_Q, const VectorX &Vector_c,
					 const VectorX &Vector_lb, const VectorX &Vector_ub)
	{
		assert(Matrix_A.rows() == Vector_b.rows() && Matrix_D.rows() == Vector_d.rows() && Matrix_A.cols() == Matrix_D.cols() &&
			   Matrix_Q.rows() == Matrix_A.cols() && Matrix_Q.cols() == Matrix_A.cols() && Vector_c.rows() == Matrix_A.cols());
		assert(Vector_lb.rows() == Matrix_A.cols() || Vector_lb.rows() == 0);
		assert(Vector_ub.rows() == Matrix_A.cols() || Vector_ub.rows() == 0);

		// 把线性不等式约束转换为C++数组
		double *A = nullptr;		/* 线性不等式约束的系数矩阵 */
		double *b = nullptr;		/* 线性不等式约束的右边值 */
		double *D = nullptr;		/* 线性等式约束的系数矩阵 */
		double *d = nullptr;		/* 线性等式约束的右边值 */
		double *Q = nullptr;		/* 二次目标值 */
		double *c = nullptr;		/* 线性目标值 */
		double *lb = nullptr;		/* variable lower bounds */
		double *ub = nullptr;		/* variable upper bounds */
		double *solution = nullptr; // 输出的决策变量的值
		double *objvalP = nullptr;	// 优化出的目标函数值

		int cols = Matrix_A.cols(); // 决策变量的个数
		bool flag_A = false, flag_D = false;
		A = new double[Matrix_A.rows() * cols]; // 允许new长度为0的数组
		b = new double[Vector_b.rows()];
		if (Matrix_A.rows() != 0)
		{
			flag_A = true;
		}

		for (size_t i = 0; i < (size_t)Matrix_A.rows(); ++i)
		{
			b[i] = Vector_b(i);
			for (size_t j = 0; j < (size_t)Matrix_A.cols(); ++j)
			{
				A[i * cols + j] = Matrix_A(i, j);
			}
		}

		// 线性等式约束
		D = new double[Matrix_D.rows() * cols];
		d = new double[Vector_d.rows()];
		if (Matrix_D.rows() != 0)
		{
			flag_D = true;
		}
		for (size_t i = 0; i < (size_t)Matrix_D.rows(); ++i)
		{
			d[i] = Vector_d(i);
			for (size_t j = 0; j < (size_t)Matrix_D.cols(); ++j)
			{
				D[i * cols + j] = Matrix_D(i, j);
			}
		}

		// 目标函数
		Q = new double[cols * cols];
		c = new double[cols];
		for (size_t i = 0; i < (size_t)cols; ++i)
		{
			c[i] = Vector_c(i);
			for (size_t j = 0; j < (size_t)cols; ++j)
			{
				Q[i * cols + j] = Matrix_Q(i, j);
			}
		}

		// 决策变量的上下限
		lb = new double[cols];
		ub = new double[cols];
		if (Vector_lb.rows() == 0)
		{
			for (size_t i = 0; i < (size_t)cols; ++i)
			{
				lb[i] = MIN_BOUND;
			}
		}
		else
		{
			for (size_t i = 0; i < (size_t)cols; ++i)
			{
				lb[i] = Vector_c(i);
			}
		}
		if (Vector_ub.rows() == 0)
		{
			for (size_t i = 0; i < (size_t)cols; ++i)
			{
				ub[i] = MAX_BOUND;
			}
		}
		else
		{
			for (size_t i = 0; i < (size_t)cols; ++i)
			{
				ub[i] = Vector_c(i);
			}
		}

		// 决策变量
		solution = new double[cols];
		// 优化目标函数值
		objvalP = new double;

		// 传入Gurobi求解API中
		ResultData result;
		result.success_ = Gurobi_dense_optimize(Matrix_A.rows(), Matrix_D.rows(), cols, A, b, D, d, Q, c, lb, ub, solution, objvalP);
		result.x.resize(cols, 1);
		for (size_t i = 0; i < (size_t)cols; ++i)
		{
			result.x(i) = solution[i];
		}
		result.cost_ = *objvalP;

		// 释放内存
		if (flag_A)
		{
			delete[] A;
			delete[] b;
		}
		if (flag_D)
		{
			delete[] D;
			delete[] d;
		}
		delete[] Q;
		delete[] c;
		delete[] lb;
		delete[] ub;
		delete[] solution;
		delete objvalP;

		return result;
	}

	bool Gurobi_dense_optimize(
		int rows_A, int rows_D,
		int cols,
		double *A, double *b,
		double *D, double *d,
		double *Q, double *c,
		double *lb, double *ub,
		double *solution, double *objvalP,
		int direction)
	{
		GRBEnv env = GRBEnv();					// 创建Environment
		env.set(GRB_IntParam_LogToConsole, 1);	// Gurobi求解log是否输出到控制台上，1 true, 0 false
		env.set("LogFile", "Gurobi_solve.log"); // 把Gurobi求解log写入文件中

		// 创建model
		GRBModel model = GRBModel(env);
		bool success = false;

		// 为model天剑决策变量
		char vtype[cols];
		for (size_t i = 0; i < (size_t)cols; i++)
		{
			vtype[i] = 'C';
		}
		GRBVar *vars = model.addVars(lb, ub, NULL, vtype, NULL, cols);

		/* 添加线性不等式约束 */

		for (size_t i = 0; i < (size_t)rows_A; ++i)
		{
			GRBLinExpr lhs = 0;
			for (size_t j = 0; j < (size_t)cols; ++j)
				if (A[i * cols + j] != 0)
					lhs += A[i * cols + j] * vars[j];
			model.addConstr(lhs, '<', b[i]); // 系数矩阵A按行排列为一个一维数组
		}
		// 添加线性等式约束
		for (size_t i = 0; i < (size_t)rows_D; ++i)
		{
			GRBLinExpr lhs = 0;
			for (size_t j = 0; j < (size_t)cols; ++j)
				if (D[i * cols + j] != 0)
					lhs += D[i * cols + j] * vars[j];
			model.addConstr(lhs, '=', d[i]); // 系数矩阵A按行排列为一个一维数组
		}

		// 创建优化目标函数
		GRBQuadExpr obj = 0;
		// 添加二次目标值
		for (size_t i = 0; i < (size_t)cols; ++i)
			for (size_t j = 0; j < (size_t)cols; ++j)
				if (Q[i * cols + j] != 0)
					obj += Q[i * cols + j] * vars[i] * vars[j];

		// 添加线性目标值
		for (size_t j = 0; j < (size_t)cols; ++j)
			obj += c[j] * vars[j];

		model.setObjective(obj, direction);

		// 把model写入文件lp中方便debug
		model.write("Gurobi_model.lp");

		try
		{
			// 使用Gurobi进行优化
			model.optimize();
		}
		catch (GRBException e)
		{
			std::cout << "Error code = " << e.getErrorCode() << std::endl;
			std::cout << e.getMessage() << std::endl;
		}
		catch (...)
		{
			std::cout << "Exception during optimization" << std::endl;
		}

		if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
		{
			*objvalP = model.get(GRB_DoubleAttr_ObjVal);
			for (size_t i = 0; i < (size_t)cols; ++i)
				solution[i] = vars[i].get(GRB_DoubleAttr_X);
			success = true;
		}

		delete[] vars; // 删除创建的变量array

		return success;
	}
}