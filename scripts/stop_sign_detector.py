#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import cv2
import numpy
import numpy as np

import cv_bridge
import rospy

from sensor_msgs.msg import Image

class StopSignDetector():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


if __name__ == "__main__":
    rospy.init_node('test_drive')
    test = StopSignDetector()
    while not rospy.is_shutdown():
        rospy.spin()