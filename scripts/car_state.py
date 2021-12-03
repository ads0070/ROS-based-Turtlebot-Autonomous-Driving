#!/usr/bin/env python
import math
import time
import rospy
from smach import State
from obstacle_detector import ObstacleDetector
from stop_sign_detector import StopSignDetector
from stop_line_detector import StopLineDetector
from blocking_bar_detector import BlockingBarDetector
from trace import Trace


class DetectedBlockingBar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.starting_lane = 0
        self.angular_v = math.pi / 2

    def execute(self, ud):
        start = self.choose_lane()

        if start == 1 or start == 2:
            blocking_bar = BlockingBarDetector()
            while not rospy.is_shutdown():
                if blocking_bar.go_sign:
                    blocking_bar.car_controller.set_velocity(0.8)
                else:
                    blocking_bar.car_controller.set_velocity(0)
                blocking_bar.car_controller.drive()
                if time.time() - blocking_bar.start_time > 0:
                    return 'success'

    def choose_lane(self):
        lane_changer = BlockingBarDetector()
        self.starting_lane = int(input("Choose your departure lane. (enter 1 or 2)"))
        start_time = time.time() + 0.5
        while not rospy.is_shutdown():
            if self.starting_lane == 1:
                return 1
            elif self.starting_lane == 2:
                lane_changer.car_controller.set_angular(-self.angular_v)
                lane_changer.car_controller.drive()
                if time.time() - start_time > 0:
                    rospy.sleep(1)
                    start_time = time.time() + 0.7
                    while True:
                        lane_changer.car_controller.set_velocity(1.0)
                        lane_changer.car_controller.set_angular(0)
                        lane_changer.car_controller.drive()
                        if time.time() - start_time > 0:
                            rospy.sleep(1)
                            start_time = time.time() + 0.5
                            while True:
                                lane_changer.car_controller.set_velocity(0.1)
                                lane_changer.car_controller.set_angular(self.angular_v)
                                lane_changer.car_controller.drive()
                                if time.time() - start_time > 0:
                                    rospy.sleep(1)
                                    return 2


class LaneTrace(State):
    def __init__(self):
        State.__init__(self, outcomes=['detected_stop_line', 'detected_stop_sign', 'detected_obstacle'])
        self.ready_obstacle = False

    def execute(self, ud):
        lane = Trace()
        stop_line = StopLineDetector()
        stop_sign = StopSignDetector()
        detect_obstacle = ObstacleDetector()

        while not rospy.is_shutdown():
            if stop_line.is_stopLine:
                return 'detected_stop_line'

            if stop_sign.is_stopSign:
                self.ready_obstacle = True
                return 'detected_stop_sign'

            if detect_obstacle.is_obstacle and self.ready_obstacle:
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


class DetectedStopLine(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'drive_straight'])
        self.stop_line_count = 0
        self.is_stop = True

    def execute(self, ud):
        lane = Trace()
        self.is_stop = True

        while not rospy.is_shutdown():
            if self.is_stop:
                self.stop_line_count += 1

                if self.stop_line_count == 4:
                    lane.car_controller.set_angular(0)
                    lane.car_controller.drive()

                if self.stop_line_count == 6:
                    lane.car_controller.set_angular(-0.1)
                    lane.car_controller.drive()

                self.is_stop = False
                rospy.sleep(3)

                if self.stop_line_count == 4 or self.stop_line_count == 6:
                    return 'drive_straight'

                if self.stop_line_count == 7:
                    return 'drive_left'

                return 'success'


class DetectedStopSign(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'lane_trace'])
        self.is_stopsign = False
        self.count_stopsign = 0

    def execute(self, ud):
        lane = Trace()
        self.count_stopsign += 1

        while not rospy.is_shutdown():
            if not self.is_stopsign:
                start_time = time.time()
            else:
                start_time = time.time() + 2

            if self.count_stopsign == 2:
                start_time = time.time() + 1

            while True:
                lane.car_controller.set_velocity(1)
                if lane.right_detect.lines is None:
                    lane.car_controller.set_angular(-0.3)

                if lane.left_detect.lines is None:
                    lane.car_controller.set_angular(0.3)

                lane.car_controller.drive()

                if time.time() - start_time > 0:
                    if self.is_stopsign:
                        if self.count_stopsign == 2:
                            return 'success'
                        return 'lane_trace'
                    break

            start_time = time.time() + 3
            while True:
                lane.car_controller.set_angular(0)
                lane.car_controller.set_velocity(0)
                lane.car_controller.drive()
                if time.time() - start_time > 0:
                    self.is_stopsign = True
                    break


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
                        lane.car_controller.set_velocity(0.3)
                        lane.car_controller.set_angular(0)
                    elif lane.right_detect.lines is None:
                        lane.car_controller.set_angular(-0.2)
                        lane.car_controller.set_velocity(1.0)
                    elif lane.left_detect.lines is None:
                        lane.car_controller.set_angular(0.2)
                        lane.car_controller.set_velocity(1.0)
                    else:
                        lane.car_controller.set_velocity(0.8)
                        lane.car_controller.set_angular(0)
                    lane.car_controller.drive()

                if time.time() - start_time > 0:
                    return 'success'


class DriveStraight(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        lane = Trace()
        while not rospy.is_shutdown():
            start_time = time.time() + 8
            while True:
                lane.car_controller.set_velocity(0.8)
                lane.car_controller.drive()
                if time.time() - start_time > 0:
                    return 'success'


class ProjectEnd(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        print "END"
        return 'success'
