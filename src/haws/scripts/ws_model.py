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
from ha_model import *
import ha_model
import numpy as np

#robot pose and twist
robot_ps = PoseStamped()
robot_tw = Twist()

FUTURE = 4.0 #time into the future for simulation
ALERT = 1.0 #time into the future for alert to get into the avoid set

def ps_callback(ps):
    'PoseStamped callback'
    global robot_ps
    robot_ps = ps

def tw_callback(tw):
    'Twist Callback'
    global robot_tw
    robot_tw = tw

# Initializes everything
def start():
    global x,y,h,v,w
    global STRAIGHT, CURVE, FUTURE, ALARM

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber('turtle1/pose', PoseStamped, ps_callback)
    rospy.Subscriber('turtle1/cmd_vel', Twist, tw_callback)

    global pub
    pub_tags = rospy.Publisher('tags', Tags, queue_size=100)
    pub_paths = rospy.Publisher('paths',Path, queue_size=100)

    # starts the node
    rospy.init_node('Dift')
    r = rospy.Rate(50)

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

        # setup simulation
        #TODO: make general
        X0 = np.array([x, y, yaw])
        u = np.array([vx, wz])

        now = rospy.get_rostime()
        t0=now.secs
        tlim=FUTURE+t0

        print '\n*** simulating: ***'
        print 't0=',t0,'tlim=',tlim

        # simulate each input combination and store the results
        simResult = \
            ha_model.h.sim(CURVE, X0, u, t0, tlim,
        debug_flag = False, haws_flag = True, Ts=0.1)
        timeToAvoid = simResult.timeToAvoid
        avoid_activated = simResult.avoid_activated
        sim_path = simResult.path()

        # calculate the tag value for each input

        dift_tags = create_tags(timeToAvoid,avoid_activated,ALERT)

        # create ROS Path message
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

        # create ROS Tags message
        tag = Tags()

        # publish tag and path to topics Tags and Path
        pub_tags.publish(tag)
        pub_paths.publish(path)

        r.sleep()

if __name__ == '__main__':
    start()


