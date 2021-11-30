#!/usr/bin/env python
import numpy
import cv_bridge
from sensor_msgs.msg import Image
from car_controller import CarController
import rospy
import cv2


class BlockingBarDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.go_sign = None
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.car_controller = CarController()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        lower_red = numpy.array([0, 0, 90])
        upper_red = numpy.array([5, 5, 110])
        img = cv2.inRange(image, lower_red, upper_red)

        h, w = img.shape
        img[0:180, 0:w] = 0
        img[240:h, 0:w] = 0

        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 3:
            self.go_sign = True
        else:
            self.go_sign = False