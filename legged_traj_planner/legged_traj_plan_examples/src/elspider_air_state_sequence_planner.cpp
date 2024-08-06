/**
 * @file elspider_air_simple_planner.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "legged_traj_plan_examples/elspider_air_planner/ElSpiderAirStateSequencePlanner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elspider_air_state_sequence_planner");
    ros::NodeHandle nh("~");
    ros::Time::init(); // FIXME: some where call ros::Time::now() before nh_ initialized

    // Load ElSpiderAirDummy
    std::string robot_interface_type;
    nh.getParam("robotInterface", robot_interface_type);
    std::shared_ptr<ElSpiderAirInterface> robot_interface;
    if (robot_interface_type == "ElSpiderAirDummy")
    {
        DummyElSpiderAirInterfaceROSConfig dummy_config;
        dummy_config.loadParam(nh, "ElSpiderAirDummy");
        robot_interface = std::make_shared<DummyElSpiderAirInterfaceROS>(dummy_config);
    }
    else if (robot_interface_type== "ElSpiderAirROS")
    {
        ElSpiderAirInterfaceROSConfig config;
        config.loadParam(nh, "ElSpiderAirROS");
        robot_interface = std::make_shared<ElSpiderAirInterfaceROS>(config);
    }
    else
    {
        ROS_ERROR("Unknown robot interface type: %s", robot_interface_type.c_str());
        return -1;
    }

    // Load ElSpiderAirShadow
    std::shared_ptr<ElSpiderAirInterface> interface_shadow;
    DummyElSpiderAirInterfaceROSConfig interface_shadow_config;
    interface_shadow_config.loadParam(nh, "ElSpiderAirShadow");
    interface_shadow = std::make_shared<DummyElSpiderAirInterfaceROS>(interface_shadow_config);

    // Load SwingTrajPlannerConfig
    SwingTrajPlannerConfig swing_traj_planner_config;
    swing_traj_planner_config.loadParams(nh);

    // Create planner
    ElSpiderAirStateSequencePlanner planner(swing_traj_planner_config,
                                            robot_interface,
                                            interface_shadow);
    planner.run();
    return 0;
}