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
un_md[0][0] = [0.2, 0.1] # v, straight
un_md[0][1] = [0.2, 0.1] # v, curve
un_md[1][0] = [0.0, 0.1] # w, straight
un_md[1][1] = [-0.2, 0.08] # w, curve

def npdf_00(u):
    mu = un_md[0][0][0]
    var= un_md[0][0][1]
    return normal_pdf(u, mu, var)/normal_pdf(mu, mu, var)

def npdf_01(u):
    mu = un_md[0][1][0]
    var= un_md[0][1][1]
    return normal_pdf(u, mu, var)/normal_pdf(mu, mu, var)

def npdf_10(u):
    mu = un_md[1][0][0]
    var= un_md[1][0][1]
    return normal_pdf(u, mu, var)/normal_pdf(mu, mu, var)

def npdf_11(u):
    mu = un_md[1][1][0]
    var= un_md[1][1][1]
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

def plot_input_distribution(L, j, e):
    import matplotlib.pyplot as plt
    fig = plt.figure()
    fig.set_size_inches(5, 5)
    xmin = -0.8
    xmax = 0.8
    x_plot = np.linspace(xmin, xmax)
    y_plot = []

    for u in x_plot:
        y_plot.append(L[j][e](u))

    plt.plot(x_plot, y_plot)
    ax = fig.add_subplot(111)
    plt.grid(True)

    plt.xlabel('Input')
    plt.ylabel('Legitimate Factor')
    ax.set_xlim([xmin, xmax])
    figdir = '/home/rafa/dift/thesis_latex/figures/'
    plt.savefig(figdir + 'inputs_dift'
                + '_' + str(j)
                + '_' + str(e)
                + '.eps',
                    format='eps', dpi=1000)

# creation of dift model
L = [[npdf_00, npdf_01], [npdf_10, npdf_11]]
dift = dift_main.DiftModel(L, un_md)

for j in range(0, 2):
    for e in range(0, 2):
        plot_input_distribution(L, j, e)


