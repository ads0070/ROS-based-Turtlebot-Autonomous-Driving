#! /usr/bin/env python

import Drive_vel

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

'''
Common used Twist publisher class
'''
class Drive:
    def __init__(self):
        self.twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def publish(self):
        #Publish
        t = Twist()
        if Drive_vel.stop:  # have to stop
            t.angular.z = 0.0
            t.linear.x = 0.0
        else:
            t.angular.z = Drive_vel.turn
            t.linear.x = Drive_vel.speed
        self.twist_pub.publish(t)

'''
Publish Twist MSG use key input
w: go
s: back
a: increse turn left
d: increse turn right
q: turn left
e: turn right
x: force stop
'''
class Drive_Keys(Drive):
    def __init__(self):
        Drive.__init__(self)
        self.key_mapping = {'w': [0, 1], 'x': [0, 0], 'q': [1, 0], 'e': [-1, 0], 's': [0, -1], 'a': [1, 1],
                            'd': [-1, 1]}
        rospy.Subscriber('keys', String, self.keys_cb, self.twist_pub)
        #rospy.spin()

    def keys_cb(self, msg, twist_pub):
        if len(msg.data) == 0 or not self.key_mapping.has_key(msg.data[0]):
            return  # Unknown Key
        vels = self.key_mapping[msg.data[0]]
        Drive_vel.turn = vels[0]
        if msg.data[0] != 'a' or msg.data != 'd':
            Drive_vel.speed = vels[1]
        self.publish()


'''
Publish Twist MSG use Method
'''
class Drive_Method(Drive):
    def __init__(self):
        Drive.__init__(self)

    def set_control(self, speed, turn):
        Drive_vel.speed = 0
        Drive_vel.turn = 0

    def forceStop(self):
        Drive_vel.speed = 0
        Drive_vel.turn = 0

    def forceTurn(self, angle):
        Drive_vel.speed = 0
        Drive_vel.turn = angle

    def forceAngle(self, angle):
        Drive_vel.speed = 0
        Drive_vel.turn = (angle * math.pi / 180.0)

    def setTurn(self, angle):
        Drive_vel.turn = angle

    def setAngle(self, angle):
        Drive_vel.turn = (angle * math.pi / 180.0)

    def setSpeed(self, speed):
        Drive_vel.speed = speed

    def straight(self):
        Drive_vel.speed = 1.0

    def stop_sign(self):
        Drive_vel.stop = True
        self.forceStop()
        self.publish()

    def go_sign(self):
        Drive_vel.stop = False
        self.straight()
        #self.publish()