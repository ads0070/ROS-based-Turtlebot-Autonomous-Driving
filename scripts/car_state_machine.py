#!/usr/bin/env python
import cv2

import cv_bridge
import rospy
from sensor_msgs.msg import Image

import car_state
from smach import StateMachine

class CarStateMachine(object):
    def __init__(self):
        self.auto_drive = StateMachine(outcomes=['success'])
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = cv_bridge.CvBridge()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Driving screen",image)
        cv2.waitKey(3)

    def drive_car(self):
        with self.auto_drive:
            StateMachine.add('DETECT_BLOCKING_BAR', car_state.DetectedBlockingBar(), transitions={'success': 'LANE_TRACE'})
            StateMachine.add('DETECT_STOP_LINE', car_state.DetectedStopLine(), transitions={'success': 'LANE_TRACE'})
            StateMachine.add('LANE_TRACE', car_state.LaneTrace(), transitions={'success': 'DETECT_BLOCKING_BAR'})
            StateMachine.add('PROJECT_END', car_state.ProjectEnd(), transitions={'success': 'success'})
        self.auto_drive.execute()

if __name__ == "__main__":
    rospy.init_node('auto_drive')
    car_state_machine = CarStateMachine()
    car_state_machine.drive_car()
    while not rospy.is_shutdown():
        rospy.spin()