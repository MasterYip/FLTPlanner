
/* related header files */
#include "cvx_trajopt/cvx_trajopt.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cvx_trajopt_node");
    ros::NodeHandle nh_;

    CVX_TrajOpt_Config config;
    ros::NodeHandle nh_priv("~");

    config.loadParameters(nh_priv);
    CVX_TrajOpt trajopt(config, nh_);

    if (config.testRate > 0.0)
    {
        ros::Rate lr(config.testRate);
        while (ros::ok())
        {
            // polyVe.conductVE();
            // polyVe.vPolyMergeTest();
            trajopt.draw_vpoly_2DinHullPointset();
            ros::spinOnce();
            lr.sleep();
        }
    }
    else
    {
        ros::spin();
    }

    return 0;
}