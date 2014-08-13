#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from racetrack import *
import racetrack
from haws.msg import Tags
from haws.msg import Dift_paths

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

# Intializes everything
def start():
    global x,y,h,v,w
    global STRAIGHT, CURVE, FUTURE, ALARM
    
    print '*** HAWS started ***'
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('dift', Tags, queue_size=50)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("turtle1/pose", Pose, pose_callback)
    # starts the node
    rospy.init_node('dift')
    r = rospy.Rate(50)
    
    while not rospy.is_shutdown():

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
        
        inputTags = create_tags(timeToAvoid,avoid_activated,ALERT)
        tag = Tags()
        tag.do_nothing = 1
        tag.do_inputs = 2
        tag.do_current = 3
        tag.path_current = [[3.13,-0.34,1.2,2],[1,2,3,5]]

        #publish tag to topic Tags
        pub.publish(tag)

        r.sleep()


def input_create_options(u):
    #TODO: make general for any number of inputs
    #hint: use tuples/
    #or something like positions = [(i,j) for i in range(5) for j in range(4)]

    v = u[0]
    w = u[1]

    p1 = [0, 0]
    p2 = [v, 0]
    p3 = [0, w]
    p4 = [v, w]

    return [p1, p2, p3, p4]


def create_tags(time_to_avoid, avoid_activated, ALERT):
    #TODO: make general
    #TODO fix number with ALERT

    p_tags = [0]*len(avoid_activated)

    for pID, pOption in enumerate(time_to_avoid):
        if avoid_activated[pID]:
            p_tags[pID] = ALERT/(pOption+ALERT/10.0)
        else:
            p_tags[pID] = 0

    do_nothing_tags = p_tags[0]
    current_tags = p_tags[3]
    first_input_tag = p_tags[1]
    second_input_tag = p_tags[2]

    v_tag = current_tags*(first_input_tag/(second_input_tag+ALERT/10.0)) - \
        do_nothing_tags
    w_tag = current_tags*(second_input_tag/(first_input_tag+ALERT/10.0)) - \
        do_nothing_tags

    print '\n------------ TAGS --------------'
    print 'v tags', v_tag
    print 'w tags', w_tag
    print '--------------------------------\n'

    return [v_tag, w_tag]

if __name__ == '__main__':
    start()
