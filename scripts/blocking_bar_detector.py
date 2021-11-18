#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy

from std_msgs.msg import Bool

from scan_image import Scan_image
from sensor_msgs.msg import Image
from car_controller import CarController
import rospy
import cv2

class BlockingBarDetector(Scan_image):
    def __init__(self):
        Scan_image.__init__(self, 'center', 0)
        self.detect = True
        self.run = False
        self.go_sign = None
        self.bar_pub = rospy.Publisher('camera/rgb/image_raw/p2_bar', Image, queue_size=1)
        self.go_sign_pub = rospy.Publisher('blocking_bar', Bool, queue_size=1)
        self.car_controller = CarController()

    def image_callback(self, msg):
        Scan_image.image_callback(self, msg)
        lower_red = numpy.array([0, 0, 90])
        upper_red = numpy.array([5, 5, 110])
        img = cv2.inRange(self.image, lower_red, upper_red)

        h, w = img.shape
        img[0:180, 0:w] = 0
        img[240:h, 0:w] = 0

        _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 3:
            self.go_sign = True
        else:
            self.go_sign = False

        ''' no.2 lane start
        # if len(contours) == 0:
        #     self.go_sign = True
        # else:
        #     self.go_sign = False
        '''