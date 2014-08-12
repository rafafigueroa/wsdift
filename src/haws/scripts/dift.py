#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from turtlesim.msg import Pose
import numpy as np
from haws import *
from racetrack import *
import racetrack

x=None
y=None
h=None
v=None
w=None

STRAIGHT = 0 
CURVE = 1 

FUTURE = 2.0 #time into the future for simulation
ALERT = 1.0 #time into the future for alert to get into the avoid set

def pose_callback(tsim_pose):
    global x,y,h,v,w
    
    x = tsim_pose.x
    y = tsim_pose.y
    h = tsim_pose.theta
    v = tsim_pose.linear_velocity
    w = tsim_pose.angular_velocity
    
    #testing
    v=0.1
    w=0.1
    

# Intializes everything
def start():
    global x,y,h,v,w
    global STRAIGHT, CURVE, FUTURE, ALARM
    
    print '*** HAWS started ***'
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist,queue_size=50)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("turtle1/pose", Pose, pose_callback)
    # starts the node
    rospy.init_node('haws')
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        print "running %s"%rospy.get_rostime()
        #simulate system with different u combinations
        uSim = input_create_options([v,w])

        X0=np.array([x,y,h])
        now = rospy.get_rostime()
        t0=now.secs
        tlim=FUTURE+t0

        print '\n*** simulating: ***'
        print 't0=',t0,'tlim=',tlim
        
        timeToAvoid = [None]*len(uSim)
        avoid_activated = [False]*len(uSim)
        pNames = \
        ['do_nothing','first_input','second_input','current_inputs']
        
        for uID,u in enumerate(uSim):
            print '\nuSim=',u,'\t',pNames[uID]

            # simulate each input combination and store the results
            simResult = \
            racetrack.h.sim(CURVE,X0,u,t0,tlim, \
            debug_flag = False,haws_flag = True, Ts=0.1)
            timeToAvoid[uID]=simResult.timeToAvoid
            avoid_activated[uID]=simResult.avoid_activated
            
        #calculate the tag value for each input
        
        inputTags = tags(timeToAvoid,avoid_activated,ALERT)
        print inputTags

        r.sleep()

if __name__ == '__main__':
    start()
