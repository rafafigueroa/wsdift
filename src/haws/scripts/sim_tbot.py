#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

#Python imports
import numpy as np

#redefinitions and global variables
sin = np.sin
cos = np.cos
pi = np.pi

x = 0.0
y = 0.0
h = 0.0 #Theta
v = 0.0 #Linear Velocity
w = 0.0 #Angular Velocity (Omega)


def twist_callback(tsim_Twist):
    global v, w

    v = tsim_Twist.linear.x
    w = tsim_Twist.angular.z


def start():
    """Simulates turtlebot movement
    and publishes new state continuously"""

    global x, y, h, v, w
    global pub

    #ROS setup
    pub = rospy.Publisher('turtle1/pose', Pose, queue_size = 1000)
    rospy.Subscriber('turtle1/cmd_vel', Twist, twist_callback)
    rospy.init_node('tsim')
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

        #ROS pose format:
        pos = Pose()
        pos.x = x
        pos.y = y
        pos.theta = h
        pos.linear_velocity = v
        pos.angular_velocity = w

        #Publish message to topic
        pub.publish(pos)
        r.sleep()

if __name__ == '__main__':
    start()
