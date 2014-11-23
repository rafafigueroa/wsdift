#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
from __future__ import division
import haws_variables

TEST_INPUT = haws_variables.test_input
ALERT = haws_variables.alert
Ts = haws_variables.Ts

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

class WarningSystem(object):
    def __init__(self, uID, m, w):
        self.uID = uID
        self.m = m
        self.w = w
        self.wl = 0.0
        self.z1 = 0.0
        self.z2 = 0.0

    def nn(self, tags, gamma):
        varsigma = tags[self.uID]

        # Second order system with 1s of settling time
        ws_input = self.m * gamma + self.w * varsigma
        dz1 = self.z2
        dz2 = -25*self.z1 -8*self.z2 + 24*ws_input
        self.z1 = self.z1 + dz1*Ts
        self.z2 = self.z2 + dz2*Ts

        # Saturation
        if self.z1 > 1:
            self.z1 = 1.0
        elif self.z1 < 0:
            self.z1 = 0.0

        return self.z1

# warning system model
m = 0.9
w = 0.5
gamma = 0.0

ws0 = WarningSystem(0, m,  w)
ws1 = WarningSystem(1, m,  w)

warning_system = [ws0, ws1]

def tags_callback(tg):
    global tags
    tags = tg

def conflict_callback(cf):
    global tc, gamma
    gamma = cf.gamma

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
            wl[uid] = ws.nn(tgs, gamma)

        warning_levels = Warning_Levels()
        warning_levels.warning_levels = wl
        # publish to topics
        pub.publish(warning_levels)

        r.sleep()

if __name__ == '__main__':
    start()
