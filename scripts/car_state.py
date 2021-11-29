#!/usr/bin/env python
# coding=utf-8

import time
import rospy
from std_msgs.msg import Bool
from smach import State

from obstacle_detector import ObstacleDetector
from stop_line_detector import StopLineDetector
from blocking_bar_detector import BlockingBarDetector
from trace import Trace


class DetectedBlockingBar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.blocking_bar_sub = rospy.Subscriber('blocking_bar', Bool)

    def execute(self, ud):
        block_detector = BlockingBarDetector()
        while True:
            if block_detector.go_sign:
                rospy.loginfo("Start")
                block_detector.car_controller.set_velocity(1)
                start_time = time.time() + 6

                while True:
                    block_detector.car_controller.drive()
                    if time.time() - start_time > 0:
                        break

                block_detector.car_controller.set_velocity(0)
                return 'success'


class LaneTrace(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'detected_stop_line', 'detected_obstacle'])

    def execute(self, ud):
        lane = Trace()
        stop_line = StopLineDetector()
        detect_obstacle = ObstacleDetector()
        test = False

        while not rospy.is_shutdown():
            if stop_line.is_stopLine:
                return 'detected_stop_line'

            if detect_obstacle.is_obstacle:
                return 'detected_obstacle'

            if lane.right_detect.lines is None and lane.left_detect.lines is None:
                lane.car_controller.set_velocity(0.5)
                lane.car_controller.set_angular(0)
            elif lane.right_detect.lines is None:
                lane.car_controller.set_angular(-0.8)
                lane.car_controller.set_velocity(0.5)
            elif lane.left_detect.lines is None:
                lane.car_controller.set_angular(0.8)
                lane.car_controller.set_velocity(0.5)
            else:
                lane.car_controller.set_velocity(0.8)
                lane.car_controller.set_angular(0)

            lane.car_controller.drive()

            if test:
                return 'success'


class DetectedStopLine(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.stop_line_count = 0

    def execute(self, ud):
        lane = Trace()
        stop_line = StopLineDetector()

        while not rospy.is_shutdown():
            if stop_line.is_stopLine and stop_line.area > 8000.0:
                self.stop_line_count += 1
                print ('stop!')
                if self.stop_line_count == 2:
                    lane.car_controller.set_angular(-0.3)
                    lane.car_controller.drive()
                rospy.sleep(1.5)
                stop_line.is_stopLine = False
                rospy.sleep(1.5)
                return 'success'


class DetectedObstacle(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        lane = Trace()
        detect_obstacle = ObstacleDetector()

        while not rospy.is_shutdown():
            start_time = time.time() + 5
            while True:
                if 0 < detect_obstacle.range_right < 1.8:
                    lane.car_controller.set_velocity(0)
                    lane.car_controller.set_angular(0)
                    lane.car_controller.drive()
                else:
                    if lane.right_detect.lines is None and lane.left_detect.lines is None:
                        lane.car_controller.set_velocity(1.0)
                        lane.car_controller.set_angular(0)
                    elif lane.right_detect.lines is None:
                        lane.car_controller.set_angular(-0.2)
                        lane.car_controller.set_velocity(1.0)
                    elif lane.left_detect.lines is None:
                        lane.car_controller.set_angular(0.2)
                        lane.car_controller.set_velocity(1.0)
                    else:
                        lane.car_controller.set_velocity(1.0)
                        lane.car_controller.set_angular(0)
                    lane.car_controller.drive()

                if time.time() - start_time > 0:
                    return 'success'

class ProjectEnd(State):

    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        print "wow"
