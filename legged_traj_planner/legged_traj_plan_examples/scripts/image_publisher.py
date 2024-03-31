#!/usr/bin/env python
from numpy import sort
import rospy
import os
import cv2
import sensor_msgs.msg

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
            self.img_files = sort([os.path.join(self.path, f) for f in os.listdir(self.path) if (f.endswith('.png') or f.endswith('.jpg'))])
        else:
            self.img_files.append(self.path)
        self.file_ptr = 0
    
    def getImgPath(self):
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


def main_program():
    rospy.init_node('image_publisher')
    imagePath = rospy.get_param('~image_path')
    topicName = rospy.get_param('~topic')
    pubRate = rospy.get_param('~rate', 1)
    image_pub = ImageGridMapPub(imagePath, topicName, pubRate)
    image_pub.run()


if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass
