#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from turtlesim.msg import Pose

#Python imports
import numpy as np
from scipy.integrate import odeint
import pyqtgraph as pg

#Haws imports
from gui_haws import *
import gui_haws

#redefinitions and global variables
sin = np.sin
cos = np.cos
pi = np.pi

x = 0.0
y = 0.0
h = 0.0 #Theta
v = 0.0 #Linear Velocity
w = 0.0 #Angular Velocity (Omega)

tsim_plot = pg.plot()

def twist_callback(tsim_Twist):
    global v, w

    v = tsim_Twist.linear.x
    w = tsim_Twist.angular.z


def start():
    """Simulates turtlebot movement
    and plots result continuously"""

    global x, y, h, v, w
    initial_time = True

    global pub
    pub = rospy.Publisher('turtle1/pose', Pose, queue_size=100)
    rospy.Subscriber('turtle1/cmd_vel', Twist, twist_callback)
    # starts the node
    rospy.init_node('tsim')
    r = rospy.Rate(100)

    delta_t = None
    previous_t = None

    print 'turtlebot simulation started'

    while not rospy.is_shutdown():

        #get time and delta_t
        now = rospy.get_rostime()
        tsim = now.secs
        if initial_time and (tsim>0):
            previous_t = tsim
            delta_t = 0
            initial_time = False
        else:
            delta_t = tsim - previous_t
            previous_t = tsim

        #simulate turtlebot dynamics
        Xcurrent = [x,y,h]
        Xnew = tbot_dynamics(Xcurrent,delta_t)
        x = float(Xnew[0])
        y = float(Xnew[1])
        h = float(Xnew[2])

        #ROS pose format:
        pos = Pose()
        pos.x = x
        pos.y = y
        pos.theta = h
        pos.linear_velocity = v
        pos.angular_velocity = w
        print 'x', x, '\t\ty', y, '\t\th', h, '\t\tv', v, '\t\tw', w

        #Publish message to topic
        pub.publish(pos)

        #Plot result
        gui_haws.GUI.update_gui()
        print 'after update'
        r.sleep()


def tbot_dX(X,t=0):
    'Turtlebot Derivative'
    global v, w, h #inputs gotten from topic

    dx=v*cos(h)
    dy=v*sin(h)
    dh=w

    dX=[dx,dy,dh]
    return dX


def tbot_dynamics(X0, delta_t):
    'Turtlebot Dynamics'
    #turtlebot derivative is time independent
    if delta_t > 0:
        T = np.linspace(0,delta_t)
        y = odeint(tbot_dX,X0,T)
        #we are interested in the last integration value
        return y[-1]
    else:
        return X0


if __name__ == '__main__':
    start()
