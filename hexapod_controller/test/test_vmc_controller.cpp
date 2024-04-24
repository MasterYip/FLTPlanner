/**
 * @file test_vmc_controller.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <hexapod_controller/vmc_controller.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_vmc_controller");
    ros::NodeHandle nh;
    VMCController controller(nh);
    // controller.run();
    while (ros::ok())
    {
    controller.test_getExpAcc();
    // controller.test_getGrf();
    // controller.test_fdbCalcGrf();
    }
    return 0;
}