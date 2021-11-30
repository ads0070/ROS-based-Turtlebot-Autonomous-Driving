#!/usr/bin/env python
import rospy
from car_controller import CarController
from right_line_detector import RightLineDetector
from left_line_detector import LeftLineDetector


class Trace():
    def __init__(self):
        self.right_detect = RightLineDetector()
        self.left_detect = LeftLineDetector()
        self.car_controller = CarController()