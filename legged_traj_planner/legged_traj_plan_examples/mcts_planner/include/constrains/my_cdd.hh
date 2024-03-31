/*
 * @Author: ptw 1515901920@qq.com
 * @Date: 2023-04-07 15:59:40
 * @LastEditors: ptw 1515901920@qq.com
 * @LastEditTime: 2023-04-07 16:03:24
 * @FilePath: /dynamic_MCTS/src/hit_spider/include/hit_spider/robot_state_transition/my_cdd.hh
 * @Description:
 */
#include "constrains/util.hh"
#include <cddlib/setoper.h>
#include <cddlib/cdd.h>

namespace Robot_State_Transition
{
    // 顶点转换为不等式
    bool vertices_to_H(const MatrixXX &vertices_set, MatrixXX &H_output, VectorX &h_output);
    bool H_to_vertices(const MatrixX3 &H_input, const VectorX &h_input, MatrixX3 &Vertices_output);
}