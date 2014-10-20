#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

from haws.msg import Tags
from haws.msg import Conflict
from haws.msg import Warning_Levels


#python imports
from ha_model import *
import ha_model
import numpy as np

#robot pose and twist
robot_ps = PoseStamped()
robot_tw = Twist()

# robot mode
qID = None
FUTURE = 2.0 #time into the future for simulation

def qID_callback(qid):
    global qID
    qID = int(qid.data)

def ps_callback(ps):
    'PoseStamped callback'
    global robot_ps
    robot_ps = ps

def tw_callback(tw):
    'Twist Callback'
    global robot_tw
    robot_tw = tw

def tags_callback(tg):
    global tags
    tags = list(tg.tags)

def conflict_callback(cf):
    global tc
    if cf.avoid_activated:
        tc = cf.tc # actual time to conflict
    else:
        tc = np.inf # time to conflict beyond finite horizon

def wl_callback(wl):
    global warning_levels
    warning_levels = list(wl.warning_levels)

path_current = Path()
def path_callback(path):
    global path_current
    path_current = path

# Initializes everything
def start():
    global x, y, h, v, w
    global qID
    global tags
    global tc

    # subscribed to everything except paths
    rospy.Subscriber('turtle1/pose', PoseStamped, ps_callback)
    rospy.Subscriber('turtle1/cmd_vel', Twist, tw_callback)
    rospy.Subscriber('q', UInt16, qID_callback)
    rospy.Subscriber('tags', Tags, tags_callback)
    rospy.Subscriber('conflict', Conflict, conflict_callback)
    rospy.Subscriber('paths', Path, path_callback)
    rospy.Subscriber('warning_levels', Warning_Levels, wl_callback)


    # starts the node
    rospy.init_node('Haws_Logger')
    hz = 50
    r = rospy.Rate(hz)
    Ts = 1.0/float(hz)

    while qID is None:
        pass

    fp = open('/home/rafa/Dropbox/aDift/logs/exp.csv', 'w+')
    fp.truncate()

    t = 0
    while not rospy.is_shutdown():

        x = robot_ps.pose.position.x
        y = robot_ps.pose.position.y
        z = robot_ps.pose.position.z

        # PoseStamped orientation is a quaternion
        # needs to be converted to roll, pitch, yaw

        (roll,pitch,yaw) = euler_from_quaternion([robot_ps.pose.orientation.x,
                                             robot_ps.pose.orientation.y,
                                             robot_ps.pose.orientation.z,
                                             robot_ps.pose.orientation.w])

        vx = robot_tw.linear.x
        vy = robot_tw.linear.y
        vz = robot_tw.linear.z
        wx = robot_tw.angular.x
        wy = robot_tw.angular.y
        wz = robot_tw.angular.z

        t += Ts


        line = [t, qID, x, y, yaw, vx, wz, tc,
                tags[0], tags[1], warning_levels[0],
                warning_levels[1]]

        line_str = []
        for c in line:
            line_str.append(str(c))

        line_joined = ', '.join(line_str)
        fp.write(line_joined)
        fp.write('\n')

        r.sleep()

    fp.close()

if __name__ == '__main__':
    start()


