#! /usr/bin/env python

import rospy
import numpy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

class Scan_image:
    def __init__(self, name, slice_type): # input = center / left / right,
        t_name = 'camera/rgb/image_raw'
        if name == 'left':
            t_name = 'my_left_camera/rgb/image_raw'
        if name == 'right':
            t_name = 'my_right_camera/rgb/image_raw'
        self.bridge = cv_bridge.CvBridge()
        self.sub = rospy.Subscriber(t_name, Image, self.image_callback)
        self.image = None
        self.mask = None
        self.hsv = None
        self.slice_type = slice_type
        self.cx = 0
        self.cy = 0

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        _, _, v = cv2.split(self.hsv)
        self.mask = cv2.inRange(v, 200, 225)
        if self.slice_type == 1:
            height, width = self.mask.shape
            self.mask[:,:width / 4] = 0
        elif self.slice_type == 2:
            height, width = self.mask.shape
            self.mask[:, width * 3 / 4:] = 0

        M = cv2.moments(self.mask)
        if M['m00'] > 0:
            self.cx = int(M['m10'] / M['m00'])
            self.cy = int(M['m01'] / M['m00'])