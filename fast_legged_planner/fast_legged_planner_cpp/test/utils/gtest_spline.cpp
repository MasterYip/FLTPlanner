#include "fast_legged_planner/utils/Spline.h"
#include <gtest/gtest.h>
#include <iostream>
#include <mgl2/mgl.h>
#include <mgl2/qt.h>

TEST(SplineTest, test_cubic_evaluate_pos)
{
    Eigen::MatrixXd para_mat(4, 4);
    para_mat << 1, 4, 1, 0,
        -3, 0, 3, 0,
        3, -6, 3, 0,
        -1, 3, -3, 1;
    para_mat /= 6;

    // Test position 1
    Eigen::MatrixXd knots(4, 1);
    knots << 1, 2, 3, 4;
    double t = 0.5;
    int d_order = 0;
    Eigen::VectorXd result = cubic_evaluate(para_mat, knots, t, d_order);
    Eigen::VectorXd expected_result(1);
    expected_result << 2.5;
    ASSERT_TRUE(result.isApprox(expected_result));
}

TEST(SplineTest, test_cubic_evaluate_pos2)
{
    Eigen::MatrixXd para_mat(4, 4);
    para_mat << 1, 4, 1, 0,
        -3, 0, 3, 0,
        3, -6, 3, 0,
        -1, 3, -3, 1;
    para_mat /= 6;

    // Test position 2
    Eigen::MatrixXd knots(4, 2);
    knots << 1, 1, 1, -1, -1, -1, -1, 1;
    double t = 0.5;
    Eigen::VectorXd result = cubic_evaluate(para_mat, knots, t);
    Eigen::VectorXd expected_result(2);
    // expected_result << 0, -0.875;
    expected_result << 0, -0.9166666667;
    std::cout << "result: " << result.transpose() << std::endl;
    std::cout << "expected_result: " << expected_result.transpose() << std::endl;
    ASSERT_TRUE(result.isApprox(expected_result, 1e-6));
}

TEST(SplineTest, test_unibspline)
{
    Eigen::MatrixXd params(4, 2);
    params << 1, 1, 1, -1, -1, -1, -1, 1;
    UniBSpline spline(params);
    ASSERT_TRUE(spline.get_dimen() == 2);

    Eigen::Vector2d expected_result;
    expected_result << 1, 1;
    std::cout << "result: " << spline.evaluate(-1).transpose() << std::endl;
    std::cout << "expected_result: " << expected_result.transpose() << std::endl;
    ASSERT_TRUE(spline.evaluate(-1).isApprox(expected_result, 1e-6));

    expected_result << 0, -0.91666667;
    std::cout << "result: " << spline.evaluate(1.5).transpose() << std::endl;
    std::cout << "expected_result: " << expected_result.transpose() << std::endl;
    ASSERT_TRUE(spline.evaluate(1.5).isApprox(expected_result, 1e-6));
}

TEST(SplineTest, test_unibspline_draw)
{
    Eigen::MatrixXd params(7, 2);
    // params << 1, 1, 1, -1, -1, -1, -1, 1;
    params << 2, 1, 1, -1, 0.5, 0, 0, 0, -0.5, 0, -1, -1, -2, 1;
    UniBSpline spline(params);

    int sample_num = 101;

    mglData x(sample_num), y(sample_num);
    for (int i = 0; i < sample_num; i++)
    {
        double t = i * (spline.get_range().second - spline.get_range().first) / (sample_num - 1) + spline.get_range().first;
        Eigen::VectorXd result = spline.evaluate(t);
        x.a[i] = result(0);
        y.a[i] = result(1);
    }
    std::cout << "t=1.5: " << spline.evaluate(1.5).transpose() << std::endl;
    // Plot
    mglQT gr;
    gr.Title("bspline");
    gr.SetOrigin(0, 0);
    gr.SetRanges(-2.2, 2.2, -1.2, 1.2);
    gr.Plot(x, y, "b-2");
    gr.Axis();
    gr.Grid();
    gr.Run();
}