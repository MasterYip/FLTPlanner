/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-03-15 13:04:19
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-04-07 22:14:57
 * @FilePath: /dynamic_MCTS/src/hit_spider/src/robot_state_transition/solve_LP_GLPK.cpp
 * @Description: GLPK求解LP问题实现
 */
#include "constrains/solve_LP_GLPK.hh"
#include "constrains/util.hh"

namespace Robot_State_Transition
{
	bool solve_LP_Bretl_GLPK(const MatrixXX &A,
							 const VectorX &b,
							 const MatrixXX &D,
							 const VectorX &d,
							 const VectorX &g,
							 VectorX &vec_opt)
	{
		assert(A.cols() != 0); // 一定要有不等式约束

		const unsigned int n_structural = A.cols();			  // GLPK中structural variables（原决策变量）的个数
		const unsigned int n_auxiliary = A.rows() + D.rows(); // GLPK中auxiliary variables（辅助变量）的个数

		const unsigned int n_unequal = A.rows(); // 不等式约束数量
		const unsigned int n_equal = D.rows();	 // 等式约束数量

		// 判断输入的是否合理
		assert(b.rows() == n_unequal && d.rows() == n_equal);
		assert(D.cols() == n_structural || D.cols() == 0);
		assert(g.rows() == n_structural || g.rows() == 0);

		// ------GLPK问题对象准备------
		glp_prob *lp;			// LP问题对象
		lp = glp_create_prob(); // 创建LP问题对象
								// glp_set_prob_name(lp, "sample"); // 为LP问题对象命名

		// LP问题优化的方向
		glp_set_obj_dir(lp, GLP_MIN);

		glp_add_rows(lp, n_auxiliary); // 等式约束的数量（辅助变量的数量）

		// ------添加决策变量------
		glp_add_cols(lp, n_structural); // 添加决策变量
		for (size_t i = 1; i <= n_structural; ++i)
		{
			glp_set_col_bnds(lp, i, GLP_FR, 0, 0); // 无约束
		}

		// ------创建GLPK的等式约束对应的矩阵------
		int ia[1 + n_auxiliary * n_structural];	   // 约束矩阵的行元素（原决策变量）
		int ja[1 + n_auxiliary * n_structural];	   // 约束矩阵的列元素（辅助变量）
		double ar[1 + n_auxiliary * n_structural]; // 约束矩阵的值

		int id_row = 1;	 // 当前处理的约束序号
		int id_col = 1;	 // 约束对应决策变量序号
		int id_temp = 1; // GLPK数组的总长度

		// 原问题的不等式约束矩阵A
		for (size_t i = 0; i < n_unequal; ++i, ++id_row)
		{
			glp_set_row_bnds(lp, id_row, GLP_UP, 0, b(i)); // 辅助变量小于等于向量b
			id_col = 1;

			for (size_t j = 0; j < n_structural; ++j, ++id_col)
			{
				if (A(i, j) != 0)
				{
					ia[id_temp] = id_row, ja[id_temp] = id_col, ar[id_temp] = A(i, j);
					++id_temp;
				}
			}
		}

		// 原问题的等式约束矩阵D
		for (size_t i = 0; i < n_equal; ++i, ++id_row)
		{
			glp_set_row_bnds(lp, id_row, GLP_FX, d(i), d(i)); // 辅助变量小于等于向量b
			id_col = 1;

			for (size_t j = 0; j < n_structural; ++j, ++id_col)
			{
				if (D(i, j) != 0)
				{
					ia[id_temp] = id_row, ja[id_temp] = id_col, ar[id_temp] = D(i, j);
					++id_temp;
				}
			}
		}

		// 设置目标值
		if (g.rows() != 0)
		{
			for (size_t i = 1; i <= n_structural; ++i)
			{
				glp_set_obj_coef(lp, i, g(i - 1)); // 决策变量的目标值
			}
		}
		else
		{
			for (size_t i = 1; i <= n_structural; ++i)
			{
				glp_set_obj_coef(lp, i, 0.0); // 无目标
			}
		}
		glp_set_obj_coef(lp, 0, 0.0);				  // 设置目标函数的常量值
		glp_load_matrix(lp, id_temp - 1, ia, ja, ar); // 获取GLPK约束矩阵

		//------求解LP问题------
		glp_smcp opts;				// LP求解器控制参数
		glp_init_smcp(&opts);		// 初始化控制参数为默认值
		opts.msg_lev = GLP_MSG_OFF; // 控制参数赋值，在求解的时候把这个参数传进去，这里可以还选择GLP_MSG_OFF

		// debug 写文件
		//  glp_write_lp(lp, NULL, "/home/ptw/now/robot_state_transition/haha.lp");

		bool output_flag = false;
		glp_simplex(lp, &opts); // 求解LP问题
		int res = glp_get_status(lp);
		if (res == GLP_OPT)
		{
			output_flag = true;
			// 提取最优决策变量
			vec_opt.resize(n_structural, 1);
			for (size_t i = 1; i <= n_structural; ++i)
			{
				vec_opt(i - 1) = glp_get_col_prim(lp, i);
			}
		}
		else if (res == GLP_UNBND)
		{
			output_flag = false; // 对于Bretl问题不可能出现无穷多解的。
			std::cout << "\033[33m\t@@@模型存在无穷多最优解!\t\033[0m" << std::endl;
		}
		else
		{
			std::cout << "\033[33m\t@@@There is a problem when solving LP problem (GLPK)!\t\033[0m" << std::endl;
			std::cout << "res(GLPK的status) = " << res << std::endl;
			output_flag = false;
		}

		// 释放GLPK使用的内存
		glp_delete_prob(lp);
		glp_free_env();

		return output_flag;
	}

	bool solve_LP_GLPK(const MatrixXX &A,
					   const VectorX &b,
					   const MatrixXX &D,
					   const VectorX &d,
					   const VectorX &g,
					   const VectorX &minBounds,
					   const VectorX &maxBounds,
					   bool direction_max,
					   VectorX *result_x,
					   double *result_optimization)
	{
		assert(A.cols() != 0); // 一定要有不等式约束

		const unsigned int n_structural = A.cols();			  // GLPK中structural variables（原决策变量）的个数
		const unsigned int n_auxiliary = A.rows() + D.rows(); // GLPK中auxiliary variables（辅助变量）的个数

		const unsigned int n_unequal = A.rows(); // 不等式约束数量
		const unsigned int n_equal = D.rows();	 // 等式约束数量

		// 判断输入的是否合理
		assert(b.rows() == n_unequal && d.rows() == n_equal);
		assert(D.cols() == n_structural || D.cols() == 0);
		assert(g.rows() == n_structural || g.rows() == 0);
		assert(minBounds.rows() == n_structural || minBounds.rows() == 0);
		assert(maxBounds.rows() == n_structural || maxBounds.rows() == 0);

		// ------GLPK问题对象准备------
		glp_prob *lp;			// LP问题对象
		lp = glp_create_prob(); // 创建LP问题对象

		// LP问题优化的方向
		if (direction_max)
		{
			glp_set_obj_dir(lp, GLP_MAX);
		}
		else
		{
			glp_set_obj_dir(lp, GLP_MIN);
		}

		glp_add_rows(lp, n_auxiliary); // 等式约束的数量（辅助变量的数量）

		// ------添加决策变量------
		glp_add_cols(lp, n_structural); // 添加决策变量
		if (minBounds.rows() != 0)
		{
			if (maxBounds.rows() != 0)
			{
				// 双边约束
				for (size_t i = 1; i <= n_structural; ++i)
				{
					glp_set_col_bnds(lp, i, GLP_DB, minBounds(i - 1), maxBounds(i - 1));
				}
			}
			else
			{
				// 最小值约束
				for (size_t i = 1; i <= n_structural; ++i)
				{
					glp_set_col_bnds(lp, i, GLP_LO, minBounds(i - 1), 0);
				}
			}
		}
		else
		{
			if (maxBounds.rows() != 0)
			{
				// 最大值约束
				for (size_t i = 1; i <= n_structural; ++i)
				{
					glp_set_col_bnds(lp, i, GLP_UP, 0, maxBounds(i - 1));
				}
			}
			else
			{
				// 无约束
				for (size_t i = 1; i <= n_structural; ++i)
				{
					glp_set_col_bnds(lp, i, GLP_FR, 0, 0);
				}
			}
		}

		// ------创建GLPK的等式约束对应的矩阵------
		int ia[1 + n_auxiliary * n_structural];	   // 约束矩阵的行元素（原决策变量）
		int ja[1 + n_auxiliary * n_structural];	   // 约束矩阵的列元素（辅助变量）
		double ar[1 + n_auxiliary * n_structural]; // 约束矩阵的值

		int id_row = 1;	 // 当前处理的约束序号
		int id_col = 1;	 // 约束对应决策变量序号
		int id_temp = 1; // GLPK数组的总长度

		// 原问题的不等式约束矩阵A
		for (size_t i = 0; i < n_unequal; ++i, ++id_row)
		{
			glp_set_row_bnds(lp, id_row, GLP_UP, 0, b(i)); // 辅助变量小于等于向量b
			id_col = 1;

			for (size_t j = 0; j < n_structural; ++j, ++id_col)
			{
				if (A(i, j) != 0)
				{
					ia[id_temp] = id_row, ja[id_temp] = id_col, ar[id_temp] = A(i, j);
					++id_temp;
				}
			}
		}

		// 原问题的等式约束矩阵D
		for (size_t i = 0; i < n_equal; ++i, ++id_row)
		{
			glp_set_row_bnds(lp, id_row, GLP_FX, d(i), d(i)); // 辅助变量小于等于向量b
			id_col = 1;

			for (size_t j = 0; j < n_structural; ++j, ++id_col)
			{
				if (D(i, j) != 0)
				{
					ia[id_temp] = id_row, ja[id_temp] = id_col, ar[id_temp] = D(i, j);
					++id_temp;
				}
			}
		}

		// 设置目标值
		if (g.rows() != 0)
		{
			for (size_t i = 1; i <= n_structural; ++i)
			{
				glp_set_obj_coef(lp, i, g(i - 1)); // 决策变量的目标值
			}
		}
		else
		{
			for (size_t i = 1; i <= n_structural; ++i)
			{
				glp_set_obj_coef(lp, i, 0.0); // 无目标
			}
		}
		glp_set_obj_coef(lp, 0, 0.0);				  // 设置目标函数的常量值
		glp_load_matrix(lp, id_temp - 1, ia, ja, ar); // 获取GLPK约束矩阵

		//------求解LP问题------
		glp_smcp opts;				// LP求解器控制参数
		glp_init_smcp(&opts);		// 初始化控制参数为默认值
		opts.msg_lev = GLP_MSG_OFF; // 控制参数赋值，在求解的时候把这个参数传进去，这里可以还选择GLP_MSG_OFF

		glp_simplex(lp, &opts); // 求解LP问题

		bool output_flag = false;
		int res = glp_get_status(lp);
		if (res == GLP_OPT)
		{
			output_flag = true;
			if (result_x != nullptr) // 提取最优决策变量
			{
				(*result_x).resize(n_structural, 1);
				for (size_t i = 1; i <= n_structural; ++i)
				{
					(*result_x)(i - 1) = glp_get_col_prim(lp, i);
				}
			}
			if (result_optimization != nullptr) // 提取最优值
			{
				*result_optimization = glp_get_obj_val(lp);
			}
		}
		else if (res == GLP_UNBND)
		{
			output_flag = true;
			//debug
			// std::cout << "\033[33m\t模型有无穷最优解\t\033[0m" << std::endl;
			if (result_optimization != nullptr) // 提取最优值，无法提取最优决策变量
			{
				*result_optimization = glp_get_obj_val(lp);
			}
		}
		else
		{
			// std::cout << "\033[33m\t@@@ (GLPK) There is a problem when solving LP problem | LP模型无解 (GLPK)!\t\033[0m" << std::endl;
			// std::cout << "res(GLPK的status) = " << res << std::endl;
			// glp_write_lp(lp, NULL, "/home/ptw/dynamic_MCTS/haha.lp");
			output_flag = false;
		}

		// 释放GLPK使用的内存
		glp_delete_prob(lp);
		glp_free_env();

		return output_flag;
	}

}
