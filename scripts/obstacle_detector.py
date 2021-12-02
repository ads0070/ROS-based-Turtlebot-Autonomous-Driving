#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import LaserScan
from car_controller import CarController


class ObstacleDetector:
    def __init__(self):
        self.range_ahead = 0
        self.range_right = 0
        self.is_obstacle = False
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.car_controller = CarController()

    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]
        self.range_right = msg.ranges[220]

        if math.isnan(self.range_ahead):
            self.range_ahead = 0
        if math.isnan(self.range_right):
            self.range_right = 0

        if 0 < self.range_ahead < 3.7:
            self.is_obstacle = True
