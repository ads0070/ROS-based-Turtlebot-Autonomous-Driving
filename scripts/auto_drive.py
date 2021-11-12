#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from bar_detector import Bar_Detector
from stop_line_detector import Stop_Line_Detector

if __name__ == "__main__":
    rospy.init_node('auto_drive_bot')
    detect_bar = Bar_Detector()
    detect_bar.run = True
    stop_line = Stop_Line_Detector()
    stop_line.run = True
    rospy.spin()
