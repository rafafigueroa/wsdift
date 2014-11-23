#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
#Python imports
import numpy as np

#redefinitions and global variables
sin = np.sin
cos = np.cos
pi = np.pi

# Robotic Agent Initial Conditions

import haws_variables

x = haws_variables.x0
y = haws_variables.y0
h = haws_variables.h0
v = 0.0 #Linear Velocity
w = 0.0 #Angular Velocity (Omega)


def tw_callback(tw):
    global v, w

    v = tw.linear.x
    w = tw.angular.z


def start():
    """Simulates turtlebot movement
    and publishes new state continuously"""

    global x, y, h, v, w
    global pub

    #ROS setup
    pub = rospy.Publisher('turtle1/pose', PoseStamped, queue_size = 1000)
    rospy.Subscriber('turtle1/cmd_vel', Twist, tw_callback)
    rospy.init_node('Robotic_Agent')
    r = rospy.Rate(100)

    #ROS main loop
    while not rospy.is_shutdown():

        #simulate turtlebot dynamics
        dx = v*cos(h)
        dy = v*sin(h)
        dh = w
        x = x + 0.01 * dx
        y = y + 0.01 * dy
        h = h + 0.01 * dh

        #ROS Pose format with Quaternions Orientation:
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = '/base_link'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0
        ps.pose.orientation.x = 0
        ps.pose.orientation.y = 0
        ps.pose.orientation.z = sin(h/2.0)
        ps.pose.orientation.w = cos(h/2.0)

        #Publish message to topic
        pub.publish(ps)
        r.sleep()

if __name__ == '__main__':
    start()
