#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
A turtle simulator must be present
like sim_tbot.py
"""
#ROS imports
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion


#Python imports
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

#global variables

#robot pose and inputs
x = 0.0
y = 0.0
h = 0.0 #Theta


def ps_callback(ps):
    global x, y, h
    x = ps.pose.position.x
    y = ps.pose.position.y
    #PoseStamped orientation is a quaternion
    #needs to be converted to theta (h)
    (roll,pitch,yaw) = euler_from_quaternion([ps.pose.orientation.x,
                                             ps.pose.orientation.y,
                                             ps.pose.orientation.z,
                                             ps.pose.orientation.w])
    h = yaw

def start():
    """Takes info from ROS topics and stores
    in the global variables for the GUI"""

    rospy.Subscriber('turtle1/pose', PoseStamped, ps_callback)
    # starts the node
    rospy.init_node('Gui')
    global GUI
    #Qt loop instead of ROS loop
    GUI.run()


class GUI_haws(object):
    def __init__(self):
        #Qt Initialization (once per application)
        self.app = QtGui.QApplication([])

        #Top level Qt Widget
        self.mw = QtGui.QWidget()

        #Plot widgets and objects
        self.sim_plot_widget = pg.PlotWidget(name = 'sim')
        self.sim_plot = self.sim_plot_widget.plot(title = 'Robot Simulation')
        self.sim_plot_widget.setRange(QtCore.QRectF(-1, -1, 4, 4))

        self.dift_plot_widget = pg.PlotWidget(name = 'dift')
        self.dift_plot = self.dift_plot_widget.plot( \
            title = 'Information Tracking')

        #Layout
        layout = QtGui.QGridLayout()
        layout.addWidget(self.sim_plot_widget,0,0)
        layout.addWidget(self.dift_plot_widget,0,1)
        self.mw.setLayout(layout)
        self.mw.show()

        #GUI Loop timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(50.0) #50 ms

        #arrow represents the current direction of a robot
        self.arrow = pg.ArrowItem(angle=180, tipAngle=30, \
                                    baseAngle=20, headLen=14, \
                                    tailLen=None, brush=None)
        self.arrow.setPos(0,0)
        self.sim_plot_widget.addItem(self.arrow)


    def update_gui(self):
        global x, y, h
        print 'updating gui','x=',x,'y=',y, 'h=',h

        #Robot representation in 2D
        self.sim_plot.setData([x], [y], symbol = 'o')
        self.arrow.setPos(x,y)
        self.arrow.setRotation(-np.degrees(h))

        #Dift plot of different test paths
        self.dift_plot.setData([0,1], [0,1])
        self.app.processEvents()

    def run(self):
        self.app.exec_()

GUI = GUI_haws()

#Start Qt Loop
if __name__ == '__main__':
    start()

