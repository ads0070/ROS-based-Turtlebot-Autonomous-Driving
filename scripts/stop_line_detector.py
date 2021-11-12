#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import numpy
from scan_image import Scan_image
from scanner import Pose_scanner
from drive import Drive_Method
from rosgraph_msgs.msg import Clock

class Stop_Line_Detector(Scan_image):
    def __init__(self):
        Scan_image.__init__(self, 'center', 0)
        self.clock_sub = rospy.Subscriber('clock', Clock, self.clock_callback)
        self.pose = Pose_scanner()
        self.position = [[0.0, 0.0]]
        self.detect_time = 0
        self.now_time = 0
        self.detect = False
        self.run = False

    def clock_callback(self, msg):
        if self.run:
            self.now_time = msg.clock.secs
            if self.detect_time + 3.0 < self.now_time:
                drive = Drive_Method()
                drive.go_sign()
                self.detect = False
            else:
                drive = Drive_Method()
                drive.stop_sign()

    def image_callback(self, msg):
        if self.run:
            Scan_image.image_callback(self, msg)
            drive = Drive_Method()
            lower_white = numpy.array([0, 0, 180])
            upper_white = numpy.array([255, 30, 255])
            self.mask = cv2.inRange(self.hsv, lower_white, upper_white)
            h, w = self.mask.shape

            self.mask[0:(h / 8) * 6, 0:w] = 0
            self.mask[(h / 8) * 7:h, 0:w] = 0

            ret, thr = cv2.threshold(self.mask, 0, 255, 0)
            _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) <= 0:
                return  #not found

            for cnt in contours:
                area = cv2.contourArea(cnt)

                #if 11000.0 < area < 13000.0 and not self.detect:  # need to find area's max range
                if 10000.0 < area and not self.detect:  # need to find area's max range
                    rospy.loginfo('catch stop_line')
                    if (self.position[-1][0] - 4.0 < self.pose.position_x < self.position[-1][0] + 4.0) and (
                            self.position[-1][1] - 4.0 < self.pose.position_x < self.position[-1][1] + 4.0):
                        drive.go_sign()
                        self.detect = False
                    else:
                        if self.detect_time + 5.0 < self.now_time:
                            drive.stop_sign()
                            self.position.append([self.pose.position_x, self.pose.position_y])
                            self.detect = True
                            self.detect_time = self.now_time
                else:
                    drive.go_sign()
                    self.detect = False