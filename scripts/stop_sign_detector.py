#!/usr/bin/env python
import cv2
import numpy as np
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from car_controller import CarController


class StopSignDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.contours = []
        self.is_stopSign = False
        self.car_controller = CarController()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 0, 90])
        upper_red = np.array([5, 5, 110])
        red_img = cv2.inRange(hsv, lower_red, upper_red)

        h, w = red_img.shape
        block_bar_mask = red_img

        block_bar_mask[0:0, 0:w] = 0
        block_bar_mask[10:h, 0:w] = 0

        block_bar_mask, self.contours, hierarchy = cv2.findContours(block_bar_mask, cv2.RETR_TREE,
                                                                    cv2.CHAIN_APPROX_SIMPLE)

        if len(self.contours) > 0:
            self.is_stopSign = True
