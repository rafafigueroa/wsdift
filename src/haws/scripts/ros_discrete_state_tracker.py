#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
#ROS imports
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path

#Python imports
import numpy as np

#global variables

#robot pose and inputs
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
    pub = rospy.Publisher('q', Int8, queue_size = 1000)
    # starts the node
    rospy.init_node('discrete_state_tracker')
    r = rospy.Rate(200)

    #ROS main loop
    while not rospy.is_shutdown():
        #Publish message to topic
        q = 3
        pub.publish(q)
        r.sleep()
#Start Loop
if __name__ == '__main__':
    start()

