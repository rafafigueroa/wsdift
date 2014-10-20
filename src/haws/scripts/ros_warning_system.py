#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from haws.msg import Tags
from haws.msg import Conflict
from haws.msg import Warning_Levels
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

#python imports
import numpy as np

# global variables
tags = None
tc = None

# ws_model.py

class WarningSystem(object):
    def __init__(self, uID, m, r, w, a, tf):
        self.uID = uID
        self.m = m
        self.r = r
        self.w = w
        self.a = a
        self.tf = tf
        self.wl = 0.0
        self.v = 0.0

    def activation_function(self, v):
        return 1.0/(1.0+np.exp(-1.0*self.a*(v-0.5)))

    def nn(self, tags, tc):
        phi = tags[self.uID]
        if tc > 0:
            b = self.m*(self.tf/tc)
        else:
            b = self.m*(self.tf/0.01)

        self.v = phi*self.w + b - self.v*self.r
        y = self.activation_function(self.v)
        return y

# warning system model
m = 0.4
r = 0.2
w = 0.4
tf = 2.0
a = 10.0

ws0 = WarningSystem(0, m, r, w, a, tf)
ws1 = WarningSystem(1, m, r, w, a, tf)

warning_system = [ws0, ws1]

def tags_callback(tg):
    global tags
    tags = tg

def conflict_callback(cf):
    global tc
    if cf.avoid_activated:
        tc = cf.tc # actual time to conflict
    else:
        tc = np.inf # time to conflict beyond finite horizon

# Initializes everything
def start():
    global tags
    global tc

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber('tags', Tags, tags_callback)
    rospy.Subscriber('conflict', Conflict, conflict_callback)

    pub = rospy.Publisher('warning_levels', Warning_Levels,
                          queue_size=100)

    # starts the node
    rospy.init_node('Warning_System')
    r = rospy.Rate(50)

    # check for inputs to be available
    if tc is None:
        tc = np.inf

    while tags is None:
        print 'ws -- waiting for inputs'
        pass

    while not rospy.is_shutdown():

        tgs = list(tags.tags)
        wl = [None, None]

        for uid, ws in enumerate(warning_system):
            wl[uid] = ws.nn(tgs, tc)

        warning_levels = Warning_Levels()
        warning_levels.warning_levels = wl
        # publish to topics
        pub.publish(warning_levels)

        print 'wl:', wl, 'v:', ws0.v, ws1.v

        r.sleep()


if __name__ == '__main__':
    start()
