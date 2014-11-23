#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
from __future__ import division

#ROS imports
import rospy
from geometry_msgs.msg import Twist
from haws.msg import Tags
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#python imports
import numpy as np

# haws imports
from ha_model import *
import ha_model
import dift_main

un_md = [[None, None], [None, None]]
un_md[0][0] = [0.2, 0.1]
un_md[0][1] = [0.2, 0.1]
un_md[1][0] = [0.0, 0.1]
un_md[1][1] = [-0.2, 0.05]

def npdf_00(u):
    mu = 0.2
    var= 0.1
    return normal_pdf(u, mu, var)/normal_pdf(mu, mu, var)

def npdf_01(u):
    mu = 0.2
    var= 0.1
    return normal_pdf(u, mu, var)/normal_pdf(mu, mu, var)

def npdf_10(u):
    mu = 0
    var= 0.1
    return normal_pdf(u, mu, var)/normal_pdf(mu, mu, var)

def npdf_11(u):
    mu = -0.2
    var= 0.05
    return normal_pdf(u, mu, var)/normal_pdf(mu, mu, var)


def normal_pdf(u, mu, var):
    mu = float(mu)
    var = float(var)
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

# creation of dift model
L = [[npdf_00, npdf_01], [npdf_10, npdf_11]]
dift = dift_main.DiftModel(L, un_md)