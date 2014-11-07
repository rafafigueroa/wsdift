#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from haws.msg import Tags
from haws.msg import Conflict
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

#python imports
from ha_model import *
import ha_model
import numpy as np

#robot pose and twist
robot_ps = PoseStamped()
robot_tw = Twist()

# robot mode
qID = None
FUTURE = 4.0 #time into the future for simulation

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

# Initializes everything
def start():
    global x, y, h, v, w
    global qID

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber('turtle1/pose', PoseStamped, ps_callback)
    rospy.Subscriber('turtle1/cmd_vel', Twist, tw_callback)
    rospy.Subscriber('q', UInt16, qID_callback)

    pub_paths = rospy.Publisher('paths', Path, queue_size=100)
    pub_tc = rospy.Publisher('conflict', Conflict, queue_size=100)

    # starts the node
    rospy.init_node('Trajectory_Prediction')
    r = rospy.Rate(50)

    while qID is None:
        pass

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
        X0 = np.array([x, y, yaw])
        u = np.array([vx, wz])

        # now = rospy.get_time()
        now = 0.0
        tlim = FUTURE+now

        print '\n*** simulating: ***'
        print 'X0=', X0
        print 't0=', now, 'tlim=', tlim

        # simulate each input combination and store the results
        simResult = \
            ha_model.h.sim(qID, X0, u, now, tlim,
        debug_flag = False, haws_flag = True, Ts=0.1)

        timeToAvoid = simResult.timeToAvoid
        avoid_activated = simResult.avoid_activated
        sim_path = simResult.path()

        print '\n*** results ***'
        print 'avoid activated\t', avoid_activated
        print 'time to avoid  \t', timeToAvoid
        print 'sim path \n', sim_path

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

        # publish to topics
        pub_paths.publish(path)

        # conflict message
        conflict = Conflict()
        conflict.avoid_activated = avoid_activated
        if avoid_activated:
            conflict.tc = timeToAvoid
        else:
            conflict.tc = 99999.0

        pub_tc.publish(conflict)

        r.sleep()


if __name__ == '__main__':
    start()


