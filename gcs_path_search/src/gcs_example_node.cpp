/**
 * @file gcs_example_node.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */

/* related header files */
#include "gcs_example/gcs_example.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcs_example_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv("~");

    GCS_Example_Config config;
    config.loadParameters(nh_priv);
    GCS_Example gcs(config, nh_);

    if (config.testRate > 0.0)
    {
        ros::Rate loop_rate(config.testRate);
        while (ros::ok())
        {
            gcs.example_run(config.exampleName);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}