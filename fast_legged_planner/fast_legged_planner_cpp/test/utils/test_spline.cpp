#include "fast_legged_planner/utils/Spline.h"
#include <gtest/gtest.h>
#include <iostream>

TEST(SplineTest, test_cubic_evaluate)
{
    Eigen::MatrixXd para_mat(4, 4);
    para_mat << 1, 4, 1, 0,
        -3, 0, 3, 0,
        3, -6, 3, 0,
        -1, 3, -3, 1;
    para_mat /= 6;
    Eigen::MatrixXd knots(4, 1);
    knots << 1, 2, 3, 4;
    double t = 0.5;
    int d_order = 0;
    Eigen::VectorXd result = cubic_evaluate(para_mat, knots, t, d_order);
    Eigen::VectorXd expected_result(1);
    expected_result << 2.5;
    // EXPECT_EQ(result, expected_result);
    std::cout << "result: " << result << std::endl;
    std::cout << "expected_result: " << expected_result << std::endl;
    ASSERT_TRUE(result.isApprox(expected_result));
}