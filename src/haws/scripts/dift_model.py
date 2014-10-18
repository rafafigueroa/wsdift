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

FUTURE = 2.0 #time into the future for simulation
ALERT = 1.0 #time into the future for alert to get into the avoid set

def ps_callback(ps):
    'PoseStamped callback'
    global robot_ps
    robot_ps = ps

def tw_callback(tw):
    'Twist Callback'
    global robot_tw
    robot_tw = tw

def create_tags():
    return 20


def normal_pdf(u, mu, var):
    return (1/(np.sqrt(var)*np.sqrt(2.0*np.pi))*np.exp(-(u-mu)**2/(2*var)))

def plot_normal_distribution():
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
