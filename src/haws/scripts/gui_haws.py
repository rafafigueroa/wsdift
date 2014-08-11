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
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

#pyqtgraph configuration
pg.setConfigOptions(antialias = True)

#global variables

#robot pose and inputs
x = 0.0
y = 0.0
h = 0.0 #Theta
v = 0.0 #Linear Velocity
w = 0.0 #Angular Velocity (Omega)

def pose_callback(pose):
    global x, y, h, v, w
    x = pose.x
    y = pose.y
    h = pose.theta
    v = pose.linear_velocity
    w = pose.angular_velocity

def start():
    """Takes info from ROS topics and stores
    in the global variables for the GUI"""


    rospy.Subscriber('turtle1/pose', Pose, pose_callback)
    # starts the node
    rospy.init_node('gui')

    #Qt loop instead of ROS loop
    GUI = GUI_haws()
    GUI.run()


class GUI_haws:
    def __init__(self):
        #Qt Initialization (once per application)
        self.app = QtGui.QApplication([])

        #Top level Qt Widget
        self.mw = QtGui.QWidget()

        #Plot widgets and objects
        sim_plot_widget = pg.PlotWidget(name = 'sim')
        self.sim_plot = sim_plot_widget.plot(title = 'Robot Simulation')
        self.sim_plot_widget.setRange(QtCore.QRectF(0, 0, 10, 10))

        haws_plot_widget = pg.PlotWidget(name = 'haws')
        self.haws_plot = haws_plot_widget.plot(title = 'Information Tracking')

        #Layout
        layout = QtGui.QGridLayout()
        layout.addWidget(sim_plot_widget,0,0)
        layout.addWidget(haws_plot_widget,0,1)
        self.mw.setLayout(layout)
        self.mw.show()

        #GUI Loop timer

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100.0) #50 ms


    def update_gui(self):
        global x, y, h, v, w
        print 'updating gui','x',x
        self.sim_plot.setData([x], [y], pen = None, symbol = 'o')
        self.haws_plot.setData([0,1], [0,1])
        self.app.processEvents()

    def run(self):
        self.app.exec_()

def gui_update_sim(x,y,h):
    'updates plot with robot simulation'
    sim_plot.setData([x, x + 0.1], [y, y+0.2])
    haws_plot.setData([x, x + 0.1], [y, y-1])
    pg.QtGui.QApplication.processEvents()
    print 'updating plot'

#Start Qt Loop
if __name__ == '__main__':
    start()

