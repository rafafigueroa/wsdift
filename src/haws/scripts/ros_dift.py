#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from haws.msg import Tags
from std_msgs.msg import UInt16
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#python imports
import numpy as np

# haws imports
from ha_model import *
import ha_model
import dift_model
from dift_model import *

global qID
qID = None

#human inputs
robot_tw = Twist()

def tw_callback(tw):
    'Twist Callback'
    global robot_tw
    robot_tw = tw

def qID_callback(qid):
    global qID
    qID = int(qid.data)

# Initializes everything
def start():
    global x,y,h,v,w
    global qID

    # subscribed to human inputs
    rospy.Subscriber('turtle1/cmd_vel', Twist, tw_callback)

    # subscribed to mode tracker
    rospy.Subscriber('q', UInt16, qID_callback)

    # publishes tags
    global pub
    pub_tags = rospy.Publisher('tags', Tags, queue_size=100)

    # starts the node
    rospy.init_node('Dift')
    r = rospy.Rate(50)

    while not rospy.is_shutdown():

        vx = robot_tw.linear.x
        vy = robot_tw.linear.y
        vz = robot_tw.linear.z
        wx = robot_tw.angular.x
        wy = robot_tw.angular.y
        wz = robot_tw.angular.z

        # format for this robot
        u = [vx, wz]
        print 'u:', u
        if qID is not None:
            # calculate the tag value for each input
            print 'qID:', qID
            tags = dift_model.dift.create_tags(qID, u)
        else:
            tags = [1, 1]
        print 'tags:', tags

        # create ROS Tags message
        ros_tags = Tags()
        ros_tags.tags = tags

        # publish tag to topic
        pub_tags.publish(ros_tags)

        r.sleep()

def plot_normal_distribution():
    # utility function
    import matplotlib.pyplot as plt
    fig = plt.figure()

    x_plot = np.linspace(-5, 5, 200)
    y_plot = []

    for x in x_plot:
        y_plot.append(normal_pdf(x, 0.2, 1))

    plt.plot(x_plot, y_plot)
    ax = fig.add_subplot(111)
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    start()


