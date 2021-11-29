#! /usr/bin/env python

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

class Scan_image:
    def __init__(self, name, slice_type):
        t_name = 'camera/rgb/image_raw'
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