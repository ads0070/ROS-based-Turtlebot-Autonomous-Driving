#!/usr/bin/env python
import numpy
import time

import cv_bridge
from sensor_msgs.msg import Image
from car_controller import CarController
import rospy
import cv2


class BlockingBarDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.success = False
        self.go_sign = True
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.car_controller = CarController()
        self.start_time = time.time() + 4

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = numpy.array([0, 30, 30])
        upper_red = numpy.array([10, 255, 130])
        img = cv2.inRange(hsv, lower_red, upper_red)
        h, w, d = image.shape

        search_top = 1
        search_bot = 3 * h / 4
        img[0:search_top, 0:w] = 0
        img[search_bot:h, 0:w] = 0
        img[0:h, 0:250] = 0

        M = cv2.moments(img)
        self.go_sign = True

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 10, (255, 0, 0), -1)
            self.go_sign = False
            self.start_time = time.time() + 4