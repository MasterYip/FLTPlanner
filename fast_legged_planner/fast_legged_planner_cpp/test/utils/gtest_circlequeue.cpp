/**
 * @file test_circlequeue.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "fast_legged_planner/utils/CircleQueue.h"
#include "gtest/gtest.h"

// int main() {
//     CircleQueue q(5);
//     for (int i = 0; i < 5; ++i) {
//         q.travel();
//         q.dequeue();
//         q.dequeue();
//     }

//     q.travel();
//     for (int i = 0; i < 2; ++i) {
//         q.enqueue(i + 4);
//     }
//     q.travel();
//     for (int i = -1; i < 6; ++i) {
//         std::cout << q.at(i) << std::endl;
//     }

//     return 0;
// }

// TEST(CircleQueue, test1) {
//     CircleQueue<int> q(5);
//     for (int i = 0; i < 5; ++i) {
//         q.travel();
//         q.dequeue();
//         q.dequeue();
//     }

//     q.travel();
//     for (int i = 0; i < 2; ++i) {
//         q.enqueue(i + 4);
//     }
//     q.travel();
//     for (int i = -1; i < 6; ++i) {
//         EXPECT_EQ(q.at(i), i + 3);
//     }
// }