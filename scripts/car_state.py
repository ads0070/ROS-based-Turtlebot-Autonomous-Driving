#!/usr/bin/env python
# coding=utf-8

import time
import rospy
from std_msgs.msg import Bool
from smach import State
from stop_line_detector import StopLineDetector
from blocking_bar_detector import BlockingBarDetector
from trace import Trace

class DetectedBlockingBar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.blocking_bar_sub = rospy.Subscriber('blocking_bar', Bool)

    def execute(self, ud):
        block_detector = BlockingBarDetector()
        print "go_sign:", block_detector.go_sign
        rate = rospy.Rate(20)
        while True:
            if block_detector.go_sign:
                rospy.loginfo("Start")
                block_detector.car_controller.set_velocity(1)
                start_time = time.time() + 5

                while True:
                    block_detector.car_controller.drive()
                    if time.time() - start_time > 0:
                        break

                block_detector.car_controller.set_velocity(0)
                return 'success'

class LaneTrace(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        lane = Trace()
        stop_line = StopLineDetector()
        rate = rospy.Rate(10)
        stop_line_count = 0

        while not rospy.is_shutdown():
            line_stop = 8500
            if stop_line.area > line_stop:
                lane.car_controller.set_velocity(0)
                lane.car_controller.set_angular(0)
                stop_line_count = stop_line_count + 1
                print "stop!"
                print "stop_line_count", stop_line_count
                if stop_line_count == 2:
                    lane.car_controller.set_angular(-0.3)
                rospy.sleep(3)

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

            if stop_line_count == 4:
                print "end-----------------------------"
                return 'success'

class DetectedStopLine(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        lane = Trace()
        stop_line = StopLineDetector()
        rate = rospy.Rate(10)
        stop_line_count = 2

        while not rospy.is_shutdown():
            print "stop_line", stop_line_count

            if stop_line.area > 8500.0:
                lane.car_controller.set_velocity(0)
                lane.car_controller.set_angular(0)
                stop_line_count = stop_line_count + 1
                print ('stop!')
                print "stop_line_cout", stop_line_count
                rospy.sleep(3)

            lane.car_controller.drive()

            if stop_line_count == 4:
                print "end-----------------------------"
                return 'success'

class ProjectEnd(State):

    def __init__(self):
        State.__init__(self, outcomes=['success'])


    def execute(self, ud):
        print "wow"
