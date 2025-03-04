#include "constrains/my_cdd.hh"

namespace Robot_State_Transition
{
    // 顶点转换为不等式
    bool vertices_to_H(const MatrixXX &vertices_set, MatrixXX &H_output, VectorX &h_output)
    {
        //------初始化------
        dd_set_global_constants();
        dd_debug = false;

        //------把vertices转换为cdd矩阵------
        dd_MatrixPtr Matrix_dd = dd_CreateMatrix((vertices_set.rows()), (vertices_set.cols() + 1));
        Matrix_dd->representation = dd_Generator; // V表示法
        Matrix_dd->numbtype = dd_Real;            // 数据类型为real

        for (dd_rowrange i = 0; i < vertices_set.rows(); ++i)
        {
            dd_set_d(Matrix_dd->matrix[i][0], 1); // 1表示V-representation中的控制点集合，0表示ray射线集合
            for (dd_rowrange j = 1; j < vertices_set.cols() + 1; ++j)
            {
                dd_set_d(Matrix_dd->matrix[i][j], vertices_set(i, j - 1));
            }
        }

        // 把V-representation（cdd矩阵表示）转换为H-representation
        dd_ErrorType err = dd_NoError;
        dd_PolyhedraPtr poly = dd_DDMatrix2Poly(Matrix_dd, &err); // cdd矩阵转换为多面体
        if (err != dd_NoError)
        {
            //debug
            // dd_WriteErrorMessages(stdout, err);
            
            dd_free_global_constants(); // 释放内存
            return false;
        }

        dd_MatrixPtr H;
        H = dd_CopyInequalities(poly);

        // 把cdd数据类型转换为Eigen矩阵
        // 获取Hx<=h中哪行是“等于号”
        std::vector<long> eq_rows;
        for (long elem = 1; elem <= (long)(H->linset[0]); ++elem)
        {
            if (set_member(elem, H->linset))
                eq_rows.push_back(elem);
        }

        int rowsize = (int)H->rowsize;
        H_output.resize(rowsize + eq_rows.size(), (int)H->colsize - 1);
        h_output.resize(rowsize + eq_rows.size());
        for (int i = 0; i < rowsize; ++i)
        {
            h_output(i) = (double)(*(H->matrix[i][0]));
            for (int j = 1; j < H->colsize; ++j)
                H_output(i, j - 1) = -(double)(*(H->matrix[i][j]));
        }

        int i = 0;
        for (std::vector<long int>::const_iterator cit = eq_rows.begin(); cit != eq_rows.end(); ++cit, ++i)
        {
            h_output(rowsize + i) = -h_output((int)((*cit) - 1));
            H_output.row(rowsize + i) = -H_output.row((int)((*cit) - 1));
        }

        // 释放内存
        dd_FreeMatrix(H);
        dd_free_global_constants();

        return true;
    }


    bool H_to_vertices(const MatrixX3 &H_input, const VectorX &h_input, MatrixX3 &Vertices_output)
    {
        assert(H_input.rows() == h_input.rows());

        //------初始化------
        dd_set_global_constants();

        //------把H,h转换为cdd矩阵
        dd_MatrixPtr Matrix_dd = dd_CreateMatrix((H_input.rows()), (H_input.cols() + 1));
        Matrix_dd->representation = dd_Inequality; // H表示法
        Matrix_dd->numbtype = dd_Real;             // 数据类型为real
        for (dd_rowrange i = 0; i < H_input.rows(); i++)
        {
            dd_set_d(Matrix_dd->matrix[i][0], h_input(i));
            for (dd_rowrange j = 1; j < H_input.cols() + 1; j++)
            {
                dd_set_d(Matrix_dd->matrix[i][j], -1 * H_input(i, j - 1));
            }
        }

        // 把H-representation（cdd矩阵表示）转换为V-representation
        dd_ErrorType err = dd_NoError;
        dd_PolyhedraPtr poly = dd_DDMatrix2Poly(Matrix_dd, &err); // cdd矩阵转换为cdd多面体对象poly
        if (err != dd_NoError)
        {
            dd_WriteErrorMessages(stdout, err);
            dd_free_global_constants(); // 释放内存
            return false;
        }

        // 获得Vertices顶点集合
        dd_MatrixPtr G;
        G = dd_CopyGenerators(poly);

        // 把cdd数据类型转换为Eigen矩阵
        int rowsize = (int)G->rowsize;
        std::vector<int> SheXian;

        for (int i = 0; i < rowsize; ++i)
        {
            if ((double)*G->matrix[i][0] != 1)
            {
                std::cout << "\033[33m\t输入的不等式约束构成不了封闭多面体\t\033[0m" << std::endl;
                SheXian.push_back(i);
            }
        }

        Vertices_output.resize(rowsize - SheXian.size(), (int)G->colsize - 1);
        for (int i = 0; i < rowsize; ++i)
        {
            auto iter = std::find(SheXian.begin(), SheXian.end(), i);
            if (iter == SheXian.end())
            {
                for (int j = 1; j < G->colsize; ++j)
                    Vertices_output(i, j - 1) = (double)(*(G->matrix[i][j]));
            }
        }

        // 释放内存
        dd_FreeMatrix(G);
        dd_free_global_constants();

        return true;
    }
}