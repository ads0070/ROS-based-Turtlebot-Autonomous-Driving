#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# BEGIN ALL
import cv2
import numpy
import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class Detector_Bar:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.bar_pub = rospy.Publisher('camera/rgb/image_raw/p2_bar', Image, queue_size=1)  # detect blockbar
        self.twist = Twist()
        self.linecount = 0

    def image_callback(self, msg):
        bar = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bar, cv2.COLOR_BGR2HSV)
        lower_red = cv2.inRange(hsv, (0, 100, 100), (5, 255, 255))
        upper_red = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
        added_red = cv2.addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0)
        h, w, d = bar.shape
        search_top = 1
        search_bot = 3 * h / 4
        added_red[0:search_top, 0:w] = 0
        added_red[search_bot:h, 0:w] = 0
        added_red[0:h, 0:250] = 0

        m = cv2.moments(added_red)
        self.twist.linear.x = 0.8
        if m['m00'] > 0:
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            cv2.circle(bar, (cx, cy), 10, (255, 0, 0), -1)
            self.twist.linear.x = 0.0
            rospy.loginfo('catch bar')
        self.cmd_vel_pub.publish(self.twist)
        bar_image_msg = self.bridge.cv2_to_imgmsg(bar, 'bgr8')
        self.bar_pub.publish(bar_image_msg)  # publish
        cv2.imshow("barimage",  bar)
        cv2.waitKey(3)



if __name__ == "__main__":
    rospy.init_node('auto_drive_bot')
    Detector_Bar()
    rospy.spin()
