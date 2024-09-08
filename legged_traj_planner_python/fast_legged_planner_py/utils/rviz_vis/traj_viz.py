#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion

# ColorRGBA color
COLOR_RED = ColorRGBA(1, 0, 0, 1)
COLOR_GREEN = ColorRGBA(0, 1, 0, 1)
COLOR_BLUE = ColorRGBA(0, 0, 1, 1)
COLOR_YELLOW = ColorRGBA(1, 1, 0, 1)
COLOR_PURPLE = ColorRGBA(1, 0, 1, 1)
COLOR_CYAN = ColorRGBA(0, 1, 1, 1)

# Marker Scale
SCALE_SMALL = Vector3(0.02, 0.02, 0.02)
SCALE_MEDIUM = Vector3(0.05, 0.05, 0.05)
SCALE_LARGE = Vector3(0.1, 0.1, 0.1)


def get_scale_vector3(scale: list):
    return Vector3(scale[0], scale[1], scale[2])

# Get Marker


def get_curve_marker(points: list, frame_id: str, namespace: str,
                     color: ColorRGBA = COLOR_RED, linewidth: float = SCALE_SMALL.x):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace

    marker.action = Marker.ADD
    marker.type = Marker.LINE_STRIP
    marker.pose.orientation.w = 1.0
    marker.scale.x = linewidth
    marker.color = color
    marker.points = [Point(x, y, z) for x, y, z in points]
    return marker


def get_points_marker(points: list, frame_id: str, namespace: str,
                      color: ColorRGBA = COLOR_RED, scale: Vector3 = SCALE_MEDIUM):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace

    marker.action = Marker.ADD
    marker.type = Marker.POINTS
    marker.pose.orientation.w = 1.0
    marker.scale = scale
    marker.color = color
    marker.points = [Point(x, y, z) for x, y, z in points]
    return marker


def get_spheres_marker(points: list, frame_id: str, namespace: str,
                       color: ColorRGBA = COLOR_RED, scale: Vector3 = SCALE_MEDIUM):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace

    marker.action = Marker.ADD
    marker.type = Marker.SPHERE_LIST
    marker.pose.orientation.w = 1.0
    marker.scale = scale
    marker.color = color
    marker.points = [Point(x, y, z) for x, y, z in points]
    return marker


class TrajViz(object):
    def __init__(self, topic_name="default_traj", frame_id="odom") -> None:
        self.pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)
        self.msg = MarkerArray()
        self.frame_id = frame_id
        self.id_cnt = 0

    def add_curve(self, points: list, namespace: str = "default_curve",
                  color: ColorRGBA = COLOR_RED,
                  linewidth: float = SCALE_SMALL.x):
        marker = get_curve_marker(
            points, self.frame_id, namespace, color, linewidth)
        marker.id = self.id_cnt
        self.id_cnt += 1
        self.msg.markers.append(marker)

    def add_points(self, points: list, namespace: str = "default_points",
                   color: ColorRGBA = COLOR_RED,
                   scale: Vector3 = SCALE_MEDIUM):
        marker = get_points_marker(
            points, self.frame_id, namespace, color, scale)
        marker.id = self.id_cnt
        self.id_cnt += 1
        self.msg.markers.append(marker)

    def add_spheres(self, points: list, namespace: str = "default_spheres",
                    color: ColorRGBA = COLOR_RED,
                    scale: Vector3 = SCALE_MEDIUM):
        marker = get_spheres_marker(
            points, self.frame_id, namespace, color, scale)
        marker.id = self.id_cnt
        self.id_cnt += 1
        self.msg.markers.append(marker)

    def publish(self):
        self.pub.publish(self.msg)
        self.msg = MarkerArray()
        self.id_cnt = 0
