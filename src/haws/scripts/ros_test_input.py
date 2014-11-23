#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

from __future__ import division
import haws_variables

TEST_INPUT = haws_variables.test_input
ALERT = haws_variables.alert

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# robot pose and inputs
x = -1.0
y = 1.0
h = 0.0 #Theta

v_base = 0.2
w_circle = -v_base
bump_collision = 0.7
bump_fix = 0.2
bump_start = bump_fix + bump_collision

w_big_circle = v_base/bump_collision
w_small_circle = v_base/bump_fix
passed_middle = False
passed_bump_start = False

def ps_callback(ps):
    global x, y, h

    x = ps.pose.position.x
    y = ps.pose.position.y
    #PoseStamped orientation is a quaternion
    #needs to be converted to theta (h)
    (roll,pitch,yaw) = euler_from_quaternion([ps.pose.orientation.x,
                                             ps.pose.orientation.y,
                                             ps.pose.orientation.z,
                                             ps.pose.orientation.w])
    h = yaw

def start():
    'Creates a test input for the robotic agent'
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 100)
    global x, y, h
    global passed_middle
    rospy.Subscriber('turtle1/pose', PoseStamped, ps_callback)

    # starts the node
    rospy.init_node('test_inputs')

    # ROS main loop
    while not rospy.is_shutdown():
        now = rospy.get_time()
        # print 't:', now

        tw = Twist()
        tw.linear.x = v_base

        if TEST_INPUT == 'circle':
            if x >= 0:
                passed_middle = True

            if passed_middle:
                if x >= 0:
                    tw.angular.z = w_circle
                else:
                    tw.angular.z = 0.0

            else:
                tw.angular.z = 0.0

        elif TEST_INPUT == 'line':
            tw.angular.z = 0.0

        elif TEST_INPUT == 'diag':
            if x < - 0.1 or x > 0.1:
                tw.angular.z = 0.0
            elif y >= (1.0 + 0.9):
                tw.angular.z = - v_base/(0.1*np.sqrt(2))

        elif TEST_INPUT == 'bump':
            if x < - bump_start or x > bump_start:
                tw.angular.z = 0.0
            elif y < (1.0 + bump_collision):
                tw.angular.z = w_big_circle
            elif y >= (1.0 + bump_collision):
                tw.angular.z = -w_small_circle

        pub.publish(tw)

if __name__ == '__main__':
    start()

