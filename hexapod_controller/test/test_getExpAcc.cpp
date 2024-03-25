/**
 * @file test_getExpAcc.cpp
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
    ros::init(argc, argv, "test_getExpAcc");
    ros::NodeHandle nh;
    VMCController controller(nh);
    while (ros::ok())
    {
        // controller.test_getExpAcc();
        controller.test_getGrf();
    }
    return 0;
}