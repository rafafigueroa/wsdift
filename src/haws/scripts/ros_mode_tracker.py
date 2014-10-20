#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
# ROS imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path

# Python imports
import numpy as np

# HAWS imports
from ha_model import *
from hasimpy import *
import ha_model

#global variables

# robot pose and inputs
x = 0.0
y = 0.0
h = 0.0 #Theta

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

    global x, y, h
    rospy.Subscriber('turtle1/pose', PoseStamped, ps_callback)
    pub = rospy.Publisher('q', UInt16, queue_size = 1000)
    # starts the node
    rospy.init_node('mode_tracker')
    r = rospy.Rate(200)

    # before the loops, grabs the initial values for qID
    qID = ha_model.h.Init_qID
    #ROS main loop
    while not rospy.is_shutdown():

        # update state from ps_callback
        X = np.array([x, y, h])
        print 'qID:', qID , 'X:', X

        # update the current mode
        qID = ha_model.h.mode_tracker_guard_check(qID, X)

        #Publish message to topic
        pub.publish(qID)
        r.sleep()


# Start Loop
if __name__ == '__main__':
    start()

