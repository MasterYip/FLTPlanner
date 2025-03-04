#!/usr/bin/env python
from numpy import sort
import numpy as np
import rospy
import os
import cv2
import sensor_msgs.msg
import yaml


class ImageGridMapPub(object):

    def __init__(self, path, topic, rate=1.0):
        self.path = path
        self.topic = topic
        self.rate = rate
        self.publisher = rospy.Publisher(
            self.topic, sensor_msgs.msg.Image, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1/self.rate), self.callback)

        # Files manage
        self.img_files = []
        if os.path.isdir(self.path):
            self.img_files = sort([os.path.join(self.path, f)
                                  for f in os.listdir(self.path) if (f.endswith('.png') or f.endswith('.jpg'))])
        else:
            self.img_files.append(self.path)
        self.file_ptr = 0

    def getImgPath(self):
        """ Scroll through the image files.
        """
        ret = self.img_files[self.file_ptr]
        self.file_ptr = (self.file_ptr + 1) % len(self.img_files)
        return ret

    def callback(self, event):
        """ Convert a image to a ROS compatible message
            (sensor_msgs.Image).
        """
        img = cv2.imread(self.getImgPath(), cv2.IMREAD_UNCHANGED)
        rosimage = sensor_msgs.msg.Image()
        if img.dtype.itemsize == 2:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr16'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra16'
            else:
                rosimage.encoding = 'mono16'
        if img.dtype.itemsize == 1:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr8'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra8'
            else:
                rosimage.encoding = 'mono8'
        rosimage.width = img.shape[1]
        rosimage.height = img.shape[0]
        rosimage.step = img.strides[0]
        rosimage.data = img.tobytes()
        rosimage.header.stamp = rospy.Time.now()
        rosimage.header.frame_id = 'map'
        self.publisher.publish(rosimage)

    def run(self):
        rospy.spin()


class TimberPileGenPub(object):

    def __init__(self, mapdef_path, topic, rate=1.0, resolution=0.025, max_height=0.5, min_height=0.0):

        self.topic = topic
        self.rate = rate
        self.resolution = resolution
        self.min_height = min_height
        self.max_height = max_height
        self.publisher = rospy.Publisher(
            self.topic, sensor_msgs.msg.Image, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1/self.rate), self.callback)

        # Timber Pile Generation Parameters
        with open(mapdef_path, 'r') as f:
            mapdef = yaml.safe_load(f)
        self.Width = np.array(mapdef['TimberPileGen']['Width'])
        if (mapdef['TimberPileGen'].get('Interval') is not None):
            self.Interval = np.array(mapdef['TimberPileGen']['Interval'])
        else:
            self.Interval = self.Width
        self.Size = np.array(mapdef['TimberPileGen']['Size'])
        self.PaddingPlaneSize = np.array(mapdef['TimberPileGen']['PaddingPlaneSize'])
        self.PaddingHeight = mapdef['TimberPileGen']['PaddingHeight']
        self.Origin = np.array(mapdef['TimberPileGen']['Origin'])
        self.HeightsCoef = mapdef['TimberPileGen']['HeightsCoef']
        self.Heights = np.array(mapdef['TimberPileGen']['Heights'])
        self.Heights.resize(self.Size)

        self.img_arr = self.generate_timber_image()

    def generate_timber_image(self):
        # coef
        self.image_size = np.array(self.PaddingPlaneSize//self.resolution + 1, dtype=int)
        self.image_origin = self.Origin - (self.image_size-1)*self.resolution/2
        self.pile_origin = self.Origin - self.Size * self.Interval / 2
        img_arr = self.PaddingHeight * np.ones(tuple(self.image_size))
        for i in range(self.image_size[0]):
            for j in range(self.image_size[1]):
                pos = [self.image_origin[0] + i*self.resolution, self.image_origin[1] + j*self.resolution]
                pile_i = int((pos[0] - self.pile_origin[0]) // self.Interval[0])
                delta_i = (pos[0] - self.pile_origin[0]) % self.Interval[0] - 0.5*self.Interval[0]
                pile_j = int((pos[1] - self.pile_origin[1]) // self.Interval[1])
                delta_j = (pos[1] - self.pile_origin[1]) % self.Interval[1] - 0.5*self.Interval[1]
                if pile_i >= 0 and pile_i < self.Size[0] and pile_j >= 0 and pile_j < self.Size[1] and \
                        abs(delta_i) <= 0.5*self.Width[0] and abs(delta_j) <= 0.5*self.Width[1]:
                    img_arr[i, j] = self.Heights[pile_i, pile_j] * self.HeightsCoef
        return img_arr

    def callback(self, event):
        """ Convert a image to a ROS compatible message
            (sensor_msgs.Image).
        """
        img0 = np.array(np.clip((self.img_arr-self.min_height)/(self.max_height-self.min_height)*65535, 0, 65535), dtype=np.uint16)
        img = np.zeros((img0.shape[0], img0.shape[1], 3), dtype=np.uint16)
        for i in range(3):
            img[:, :, i] = img0
        rosimage = sensor_msgs.msg.Image()
        if img.dtype.itemsize == 2:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr16'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra16'
            else:
                rosimage.encoding = 'mono16'
        if img.dtype.itemsize == 1:
            if len(img.shape) == 3:
                if img.shape[2] == 3:
                    rosimage.encoding = 'bgr8'
                if img.shape[2] == 4:
                    rosimage.encoding = 'bgra8'
            else:
                rosimage.encoding = 'mono8'
        rosimage.width = img.shape[1]
        rosimage.height = img.shape[0]
        rosimage.step = img.strides[0]
        rosimage.data = img.tobytes()
        rosimage.header.stamp = rospy.Time.now()
        rosimage.header.frame_id = 'map'
        self.publisher.publish(rosimage)

    def run(self):
        rospy.spin()


def img_publisher():

    imagePath = rospy.get_param('~image_path')
    topicName = rospy.get_param('~topic')
    pubRate = rospy.get_param('~rate', 1)
    image_pub = ImageGridMapPub(imagePath, topicName, pubRate)
    image_pub.run()


def timber_pile_generator():
    mapdef_path = rospy.get_param('~map_def_path')
    topicName = rospy.get_param('~topic')
    pubRate = rospy.get_param('~rate', 1)
    resolution = rospy.get_param('~resolution', 0.025)  # Use 0.025m as default resolution
    min_height = rospy.get_param('~min_height', 0.0)  # Use 0.0m as default min height
    max_height = rospy.get_param('~max_height', 0.5)  # Use 0.5m as default max height

    image_pub = TimberPileGenPub(mapdef_path, topicName, pubRate, resolution, max_height, min_height)
    image_pub.run()


if __name__ == '__main__':
    rospy.init_node('image_publisher')

    try:
        if os.path.isfile(rospy.get_param('~map_def_path', "")):
            timber_pile_generator()
        elif os.path.isfile(rospy.get_param('~image_path', "")):
            img_publisher()
        else:
            rospy.logerr('Invalid image path or map definition path')
    except rospy.ROSInterruptException:
        pass
