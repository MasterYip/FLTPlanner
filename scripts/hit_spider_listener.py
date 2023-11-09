#!/usr/bin/env python
#coding=utf-8
import rospy
import math
import fast_legged_planner_py
#导入mgs
from fast_legged_planner.msg import hexapod_State


def callback(msg):
    rospy.loginfo('hexapod_State: distance=',)
    # rospy.loginfo("当前statesList的存储值为:%d", (int)statesList.size());
    rospy.loginfo("接收到消息,质心位置为:%.4f,%.4f,%.4f", msg.base_Pose_Now.position.x, msg.base_Pose_Now.position.y, msg.base_Pose_Now.position.z)
    rospy.loginfo("接收到消息,第一条腿的位置:%.4f,%.4f,%.4f", msg.feetPositionNow.foot[0].x, msg.feetPositionNow.foot[0].y, msg.feetPositionNow.foot[0].z)
    rospy.loginfo("接收到消息,当前腿的支撑状态:%d,%d,%d,%d,%d,%d", msg.support_State_Now[0], msg.support_State_Now[1], msg.support_State_Now[2], msg.support_State_Now[3], msg.support_State_Now[4], msg.support_State_Now[5])

def listener():
    rospy.init_node('hit_spider_listener', anonymous=False)
    rospy.Subscriber('supportStateTopic', hexapod_State, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()