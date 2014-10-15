#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from haws.msg import Tags
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#python imports
from racetrack import *
import racetrack
import numpy as np

#robot pose and twist
robot_ps = PoseStamped()
robot_tw = Twist()

STRAIGHT = 0 
CURVE = 1 

FUTURE = 2.0 #time into the future for simulation
ALERT = 1.0 #time into the future for alert to get into the avoid set

def ps_callback(ps):
    global robot_ps
    robot_ps = ps

def tw_callback(tw):
    global robot_tw
    robot_tw = tw

# Intializes everything
def start():
    global x,y,h,v,w
    global STRAIGHT, CURVE, FUTURE, ALARM
    
    global pub
    pub_tags = rospy.Publisher('tags', Tags, queue_size=100)
    pub_paths = rospy.Publisher('paths',Path, queue_size=100)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber('turtle1/pose', PoseStamped, ps_callback)
    rospy.Subscriber('turtle1/cmd_vel', Twist, tw_callback)

    # starts the node
    rospy.init_node('Dift')
    r = rospy.Rate(50)
    
    while not rospy.is_shutdown():

        #TODO: makegeneral
        x = robot_ps.pose.position.x
        y = robot_ps.pose.position.y
        #PoseStamped orientation is a quaternion
        #needs to be converted to theta (h)
        (roll,pitch,yaw) = euler_from_quaternion([robot_ps.pose.orientation.x,
                                             robot_ps.pose.orientation.y,
                                             robot_ps.pose.orientation.z,
                                             robot_ps.pose.orientation.w])
        h = yaw

        v = robot_tw.linear.x
        w = robot_tw.angular.z


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
        ['do_nothing','scaled_inputs','another','current_inputs']
        
        for uID,u in enumerate(uSim):
            print '\nuSim=',u,'\t',pNames[uID]

            # simulate each input combination and store the results
            simResult = \
            racetrack.h.sim(CURVE,X0,u,t0,tlim, \
            debug_flag = False, haws_flag = True, Ts=0.1)
            timeToAvoid[uID]=simResult.timeToAvoid
            avoid_activated[uID]=simResult.avoid_activated
            sim_path = simResult.path()
        #calculate the tag value for each input
        
        inputTags = create_tags(timeToAvoid,avoid_activated,ALERT)
        tag = Tags()
        tag.do_nothing = 1
        tag.do_inputs = 2
        tag.do_current = 3

        path = Path()
        path.header.frame_id = '/base_link'
        path.header.stamp = rospy.Time.now()

        xcoords = []
        ycoords = []
        for coord in sim_path:
            xcoords.append(coord[0])
            ycoords.append(coord[1])

        num_points = len(xcoords)
        print '-------------------'

        for i in range(0,num_points):
            ps = PoseStamped()
            ps.header.stamp = path.header.stamp
            ps.header.frame_id = path.header.frame_id
            ps.pose.position.x = xcoords[i]
            ps.pose.position.y = ycoords[i]
            ps.pose.position.z = 0
            ps.pose.orientation.x = 0
            ps.pose.orientation.y = 0
            ps.pose.orientation.z = robot_ps.pose.orientation.z
            ps.pose.orientation.w = robot_ps.pose.orientation.w
            path.poses.append(ps)

        #publish tag to topic Tags
        pub_tags.publish(tag)
        pub_paths.publish(path)

        r.sleep()

def create_tags2(u):
    'Tag Value Calculation'
    from ws_model import WS
    ws = WS()

    #The model spourious matrix (S)
    #scales the current input values



def input_create_options(u):
    #TODO: makegeneral
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

    #print 'v tags', v_tag
    #print 'w tags', w_tag

    return [v_tag, w_tag]

if __name__ == '__main__':
    start()
