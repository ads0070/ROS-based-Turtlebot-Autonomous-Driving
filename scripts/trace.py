#!/usr/bin/env python

import rospy
from car_controller import CarController
from right_line_detector import RightLineDetector
from left_line_detector import LeftLineDetector
from stop_line_detector import StopLineDetector

class Trace():
    def __init__(self):
        self.right_detect = RightLineDetector()
        self.left_detect = LeftLineDetector()
        self.stop_line_detect = StopLineDetector()
        self.car_controller = CarController()
        self.rate = rospy.Rate(20)

    def go_line(self):
        if self.right_detect.lines is None and self.left_detect.lines is None:
            self.car_controller.set_velocity(0.5)
            self.car_controller.set_angular(0)
        elif self.right_detect.lines is None:
            self.car_controller.set_angular(-0.8)
            self.car_controller.set_velocity(0.4)
        elif self.left_detect.lines is None:
            self.car_controller.set_angular(0.8)
            self.car_controller.set_velocity(0.4)
        else:
            self.car_controller.set_velocity(0.8)
            self.car_controller.set_angular(0)