#!/usr/bin/env python
# coding=utf-8

import rospy
from fast_legged_planner_py.perception_interface.gridmap_interface_ros import GridMap_Interface


if __name__ == '__main__':
    try:
        rospy.init_node('GridMap_Interface_py', anonymous=True)
        gmI = GridMap_Interface("grid_map")
        while (rospy.is_shutdown() is False):
            gmI.update()
            # print(gmI.grid_map.getLayers())
            try:
                # print(gmI.grid_map.get('3D_feeling'))
                print(gmI.grid_map.at('3D_feeling', [0, 0]))
            except:
                print("Layer not found!")
            rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
