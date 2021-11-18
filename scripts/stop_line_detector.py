#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import cv_bridge
import rospy
import cv2
import numpy

from sensor_msgs.msg import Image

from scan_image import Scan_image
from scanner import Pose_scanner

class StopLineDetector():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.stop_count = 0
        self.area = 0

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w = mask.shape

        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0

        _, thr = cv2.threshold(mask, 127, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) <= 0:
            return  # not found

        cnt = contours[0]
        self.area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        mask = cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 0, 255), 2)

        cv2.drawContours(mask, [cnt], 0, (255, 255, 0), 1)

        '''
        for cnt in contours:
            area = cv2.contourArea(cnt)

            if 10000.0 < area and not self.detect:
                rospy.loginfo('catch stop_line')
                self.detect = True

                # if (self.position[-1][0] - 4.0 < self.pose.position_x < self.position[-1][0] + 4.0) and \
                #         (self.position[-1][1] - 4.0 < self.pose.position_x < self.position[-1][1] + 4.0):
                #     self.detect = False
                # else:
                #     if self.detect_time + 5.0 < self.now_time:
                #         self.position.append([self.pose.position_x, self.pose.position_y])
                #         self.detect = True
                #         self.detect_time = self.now_time
            else:
                self.detect = False
        '''

# if __name__ == '__main__':
#     rospy.init_node('stop_line_finder')
#     detect_stop_line = StopLineDetector()
#     rospy.spin()