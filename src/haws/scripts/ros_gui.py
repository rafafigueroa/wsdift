#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
A turtle simulator must be present
like sim_tbot.py
"""
#ROS imports
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from haws.msg import Tags
from haws.msg import Conflict
from haws.msg import Warning_Levels
from std_msgs.msg import UInt16

#Python imports
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

#Haws imports
import dift_model
from dift_model import *

#robot pose and inputs
x = -1.0
y = 1.0
h = 0.0 #Theta
v = 0.0
w = 0.0
gamma = 0.0
Psi = [0.0, 0.0]
qID = None

#robot pose and twist
robot_ps = PoseStamped()
robot_tw = Twist()

def rx(xp, yp):
    rotang = - h - np.pi*0.5
    return np.cos(rotang) * (x - xp) - \
           np.sin(rotang) * (y - yp)

def ry(xp, yp):
    rotang = - h - np.pi*0.5
    return np.sin(rotang) * (x - xp) + \
           np.cos(rotang) * (y - yp)

def usat(val):
    'Unity saturation'
    if val > 1:
        return 1
    elif val < 0:
        return 0
    else:
        return val

def wl_callback(wl):
    global Psi
    Psi = list(wl.warning_levels)

def tags_callback(tg):
    global tags
    tags = list(tg.tags)

def conflict_callback(cf):
    global tc, gamma
    gamma = cf.gamma
    if cf.avoid_activated:
        tc = cf.tc # actual time to conflict
    else:
        tc = np.inf # time to conflict beyond finite horizon

def tw_callback(tw):
    'Twist Callback'
    global v, w
    v = tw.linear.x
    w = tw.angular.z


def qID_callback(qid):
    global qID
    qID = int(qid.data)

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

path_current = Path()
tag_ranges = dift_model.dift.tag_ranges()
nID = 0 # close to nominal
anID = 1 # almost nominal

xvn = 0.0
xva = 0.8
xwa = 0.8*3
xwn = 0.8*4

def path_callback(path):
    global path_current
    path_current = path

def start():
    """Takes info from ROS topics and stores
    in the global variables for the GUI"""


    # subscribed to human inputs
    rospy.Subscriber('turtle1/pose', PoseStamped, ps_callback)
    rospy.Subscriber('turtle1/cmd_vel', Twist, tw_callback)
    rospy.Subscriber('q', UInt16, qID_callback)
    rospy.Subscriber('tags', Tags, tags_callback)
    rospy.Subscriber('conflict', Conflict, conflict_callback)
    rospy.Subscriber('paths', Path, path_callback)
    rospy.Subscriber('warning_levels', Warning_Levels, wl_callback)
    rospy.Subscriber('paths',Path,path_callback)

    # starts the node
    rospy.init_node('Gui')
    global GUI
    #Qt loop instead of ROS loop
    GUI.run()

class GUI_haws(object):
    def __init__(self):
        #Qt Initialization (once per application)
        self.app = QtGui.QApplication([])

        #Top level Qt Widget, Main Window
        self.mw = QtGui.QWidget()
        self.mw.resize(1024, 768) # 1400, 1050

        #Plot widgets and objects
        #qreal x, qreal y, qreal width, qreal height

        self.main_plot_widget = pg.PlotWidget(name = 'main')
        self.main_plot = self.main_plot_widget.plot(
            title = 'Main GUI')
        self.main_plot.setPen(pg.mkPen(color = 'w', width = 3))
        self.main_plot_widget.setRange(QtCore.QRectF(-2, -0.33, 4, 2))

        self.racetrack_plot_widget = pg.PlotWidget(name = 'racetrack')
        self.racetrack_plot = self.racetrack_plot_widget.plot( \
            title = 'Racetrack')
        self.racetrack_plot.setPen(pg.mkPen(color = 'w', width = 3))
        self.racetrack_plot_widget.setRange(QtCore.QRectF(-8, -2, 16, 4))

        #Layout
        layout = QtGui.QGridLayout(self.mw)
        layout.addWidget(self.main_plot_widget,0,0)
        layout.addWidget(self.racetrack_plot_widget,1,0)
        layout.setRowStretch(0, 2)
        layout.setRowStretch(1, 1)

        self.mw.setLayout(layout)
        self.mw.show()

        #GUI Loop timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100.0) #50 ms

        #arrow represents the current direction of a robot
        self.arrow_sim = pg.ArrowItem(angle=180, tipAngle=30, \
                                    baseAngle=20, headLen=24, \
                                    tailLen=None, brush=0.7)

        self.arrow_sim.setPos(0,0)
        self.arrow_sim.setPen(pg.mkPen(0.9))
        self.racetrack_plot_widget.addItem(self.arrow_sim)

        self.arrow_fix = pg.ArrowItem(angle=90, tipAngle=30, \
                                    baseAngle=20, headLen=24*3, \
                                    tailLen=None, brush=0.7)

        self.arrow_fix.setPos(0,0)
        self.arrow_fix.setPen(pg.mkPen(0.9))
        self.main_plot_widget.addItem(self.arrow_fix)

        # Main GUI Graphical Items:
        self.pgplot = self.main_plot_widget.getPlotItem()
        self.rtplot = self.racetrack_plot_widget.getPlotItem()

        # RACETRACK ARC
        lw = 1
        self.garc = [[None, None],[None, None]]
        self.garcpoints = [[None, None],[None, None]]

        for p in range(0, 2):
            self.garcpoints[p][0] = [1, -1, 2, 2]
            self.garcpoints[p][1] = [-3, -1, 2, 2]

        for p in range(0, 2):
            for a in range(0, 2):
                self.garc[p][a] = pg.QtGui.QGraphicsEllipseItem(
                                    self.garcpoints[p][a][0],
                                    self.garcpoints[p][a][1],
                                    self.garcpoints[p][a][2],
                                    self.garcpoints[p][a][3])
                self.garc[p][a].setPen((pg.mkPen('g', width = lw,
                                  style=QtCore.Qt.DashLine)))
        for p in range(0, 2):
            self.garc[p][0].setStartAngle(int(5760*(0-0.25)))
            self.garc[p][1].setStartAngle(int(5760*(0-0.25)))
            self.garc[p][0].setSpanAngle(int(5760* 0.5))
            self.garc[p][1].setSpanAngle(-int(5760* 0.5))

        for a in range(0, 2):
            self.pgplot.addItem(self.garc[0][a])

        for a in range(0, 2):
            self.rtplot.addItem(self.garc[1][a])

        # Lines

        ln = 8
        self.lines = [[None]*ln, [None]*ln]
        self.linepoints = [[None]*ln, [None]*ln]
        self.linecolors = [[None]*ln, [None]*ln]

        for p in range(0, 2):
            self.linepoints[p][0] = [-4, 2, 4, 2]
            self.linepoints[p][1] = [4, 2, 4, -2]
            self.linepoints[p][2] = [4, -2, -4, -2]
            self.linepoints[p][3] = [-4, -2, -4, 2]
            self.linepoints[p][4] = [-2, 1, 2, 1]
            self.linepoints[p][5] = [-2, -1, 2, -1]
            self.linepoints[p][6] = [-2, 1, -2, -1]
            self.linepoints[p][7] = [2, 1, 2, -1]

            self.linecolors[p][0] = 'r'
            self.linecolors[p][1] = 'r'
            self.linecolors[p][2] = 'r'
            self.linecolors[p][3] = 'r'
            self.linecolors[p][4] = 'g'
            self.linecolors[p][5] = 'g'
            self.linecolors[p][6] = 'k'
            self.linecolors[p][7] = 'k'


        for p in range(0, 2):
            for l in range(0, 8):
                self.lines[p][l] = pg.QtGui.QGraphicsLineItem(
                                    self.linepoints[p][l][0],
                                    self.linepoints[p][l][1],
                                    self.linepoints[p][l][2],
                                    self.linepoints[p][l][3])

                self.lines[p][l].setPen(pg.mkPen(self.linecolors[p][l],
                                        width = lw,
                                        style=QtCore.Qt.DashLine))

        for p in range(0, 2):
            for l in range(6, 8):
                self.lines[p][l].setPen(pg.mkPen(self.linecolors[p][l],
                                        width = lw+1,
                                        style=QtCore.Qt.SolidLine))

        for l in range(0, ln):
            self.pgplot.addItem(self.lines[0][l])

        for l in range(0, ln):
            self.rtplot.addItem(self.lines[1][l])

        # QGraphicsEllipseItem ( qreal x, qreal y, qreal width, qreal height, QGraphicsItem * parent = 0 )
        self.v_indicator_frame = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1 + xvn, -0.3, 0.6, 0.6)
        self.w_indicator_frame = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1 + xwn, -0.3, 0.6, 0.6)

        self.v_indicator_frame.setPen(pg.mkPen(color = 'w', width = 3))
        self.v_indicator_frame.setBrush(pg.mkBrush(0.15))
        self.w_indicator_frame.setPen(pg.mkPen(color = 'w', width = 3))
        self.w_indicator_frame.setBrush(pg.mkBrush(0.15))
        self.pgplot.addItem(self.v_indicator_frame)
        self.pgplot.addItem(self.w_indicator_frame)

        self.v_label = pg.TextItem('v', anchor = (0.5, 0.5))
        self.w_label = pg.TextItem('w', anchor = (0.5, 0.5))
        self.v_label.setPos(-1.2, -0.3)
        self.w_label.setPos(1.2, -0.3)

        self.main_plot_widget.addItem(self.v_label)
        self.main_plot_widget.addItem(self.w_label)
        # 	QGraphicsEllipseItem ( qreal x, qreal y, qreal width, qreal height, QGraphicsItem * parent = 0 )
        self.v_indicator_wedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1 + xvn, -0.3, 0.6, 0.6)
        self.v_indicator_anwedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1 + xvn, -0.3, 0.6, 0.6)

        self.w_indicator_wedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1 + xwn, -0.3, 0.6, 0.6)
        self.w_indicator_anwedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1 + xwn, -0.3, 0.6, 0.6)

        self.vc_wedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.2 + xva, -0.2, 0.4, 0.4)
        self.vs_wedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.2 + xva, -0.2, 0.4, 0.4)
        self.va_wedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1 + xva, -0.3, 0.6, 0.6)
        self.wc_wedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.2 + xwa, -0.2, 0.4, 0.4)
        self.ws_wedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.2 + xwa, -0.2, 0.4, 0.4)
        self.wa_wedge = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1 + xwa, -0.3, 0.6, 0.6)

        self.v_indicator_wedge.setPen(pg.mkPen(None))
        self.v_indicator_anwedge.setPen(pg.mkPen(None))
        self.w_indicator_wedge.setPen(pg.mkPen(None))
        self.w_indicator_anwedge.setPen(pg.mkPen(None))

        self.vc_wedge.setPen(pg.mkPen(None))
        self.vs_wedge.setPen(pg.mkPen(None))
        self.va_wedge.setPen(pg.mkPen(None))
        self.wc_wedge.setPen(pg.mkPen(None))
        self.ws_wedge.setPen(pg.mkPen(None))
        self.wa_wedge.setPen(pg.mkPen(None))
        self.vc_wedge.setBrush(pg.mkBrush(255, 0, 0)) # red
        self.vs_wedge.setBrush(pg.mkBrush(255, 255, 0)) # yellow
        self.va_wedge.setBrush(pg.mkBrush(255, 128, 0)) # orange
        self.wc_wedge.setBrush(pg.mkBrush(255, 0, 0)) # red
        self.ws_wedge.setBrush(pg.mkBrush(255, 255, 0)) # yellow
        self.wa_wedge.setBrush(pg.mkBrush(255, 128, 0)) # orange

        # top of the pie = -int(5760*0.25)
        # SpanAngle is positive clockwise

        uID = 0 # v
        qID = 0 # not the global

        # v = linear velocity CREATION
        v_anrange_min = tag_ranges[uID][qID][anID][0]
        v_anrange_max = tag_ranges[uID][qID][anID][1]
        v_anrange_span = v_anrange_max - v_anrange_min
        v_anrange_start = v_anrange_min
        self.v_indicator_anwedge.setStartAngle(
            int(5760*(v_anrange_start-0.25)))
        self.v_indicator_anwedge.setSpanAngle(
            int(5760*v_anrange_span))
        self.v_indicator_anwedge.setBrush(pg.mkBrush(100, 220, 0))

        v_range_min = tag_ranges[uID][qID][nID][0]
        v_range_max = tag_ranges[uID][qID][nID][1]
        v_range_span = v_range_max - v_range_min
        v_range_start = v_range_min
        self.v_indicator_wedge.setStartAngle(int(5760*(v_range_start-0.25)))
        self.v_indicator_wedge.setSpanAngle(int(5760*v_range_span))
        self.v_indicator_wedge.setBrush(pg.mkBrush('g'))

        # w = omega, angular velocity CREATION
        uID = 1 # 2
        w_anrange_min = tag_ranges[uID][qID][anID][0]
        w_anrange_max = tag_ranges[uID][qID][anID][1]
        w_anrange_span = w_anrange_max - w_anrange_min
        w_anrange_start = w_anrange_min
        self.w_indicator_anwedge.setStartAngle(
            int(5760*(w_anrange_start-0.25)))
        self.w_indicator_anwedge.setSpanAngle(
            int(5760*w_anrange_span))
        self.w_indicator_anwedge.setBrush(pg.mkBrush(100, 220, 0))

        w_range_min = tag_ranges[uID][qID][nID][0]
        w_range_max = tag_ranges[uID][qID][nID][1]
        w_range_span = w_range_max - w_range_min
        w_range_start = w_range_min
        self.w_indicator_wedge.setStartAngle(int(5760*(w_range_start-0.25)))
        self.w_indicator_wedge.setSpanAngle(int(5760*w_range_span))
        self.w_indicator_wedge.setBrush(pg.mkBrush('g'))

        self.pgplot.addItem(self.v_indicator_anwedge)
        self.pgplot.addItem(self.v_indicator_wedge)
        self.pgplot.addItem(self.w_indicator_anwedge)
        self.pgplot.addItem(self.w_indicator_wedge)
        self.pgplot.addItem(self.va_wedge)
        self.pgplot.addItem(self.vc_wedge)
        self.pgplot.addItem(self.vs_wedge)
        self.pgplot.addItem(self.wa_wedge)
        self.pgplot.addItem(self.wc_wedge)
        self.pgplot.addItem(self.ws_wedge)

        # v needle CREATION
        self.v_needle = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.1, -0.3 + 0.3, 0.6, 0.6)
        self.v_needle.setPen(pg.mkPen(None))
        v = 0 # not the global
        vnt = 0.02
        r = 0.3
        # alpha = angle (radians) measured like QgraphicsEllipseItem
        alpha = (v - 0.25)*2*np.pi
        # the needle tip will be the center of the ellipse
        # angle in QGE goes in opposite direction of mathematics
        vnx = r * np.cos(-alpha)
        vny = r * np.sin(-alpha)
        # QGE box goes translated below and left of center
        vnbox_x = vnx - r
        vnbox_y = vny - r
        # Updating box
        # center offset -2 + 0.1, -0.3
        self.v_needle.setRect(vnbox_x -2 + 0.1 + r, vnbox_y, r*2, r*2)
        self.v_needle.setStartAngle(int(5760*(v - vnt/2.0 +0.25)))
        self.v_needle.setSpanAngle(int(5760*vnt))
        self.v_needle.setBrush(pg.mkBrush('r'))
        self.pgplot.addItem(self.v_needle)

        # w needle CREATION
        self.w_needle = pg.QtGui.QGraphicsEllipseItem(
            -2 + 0.9, -0.3 + 0.3, 0.6, 0.6)
        self.w_needle.setPen(pg.mkPen(None))
        w = 0 # not the global
        wnt = 0.02
        r = 0.3
        # alpha = angle (radians) measured like QgraphicsEllipseItem
        alpha = (w - 0.25)*2*np.pi
        # the needle tip will be the center of the ellipse
        # angle in QGE goes in opposite direction of mathematics
        wnx = r * np.cos(-alpha)
        wny = r * np.sin(-alpha)
        # QGE box goes translated below and left of center
        wnbox_x = wnx - r
        wnbox_y = wny - r
        # Updating box
        # center offset -2 + 0.1, -0.3
        self.w_needle.setRect(wnbox_x -2 + 0.1 + xwn + r, wnbox_y, r*2, r*2)
        self.w_needle.setStartAngle(int(5760*(w - wnt/2.0 +0.25)))
        self.w_needle.setSpanAngle(int(5760*wnt))
        self.w_needle.setBrush(pg.mkBrush('r'))
        self.pgplot.addItem(self.w_needle)

    def update_gui(self):

        global qID
        global tags
        global tc

        #Robot representation in 2D
        #self.sim_plot.setData([x], [y], symbol = 'o')

        #Arrows orientation and position
        self.arrow_sim.setPos(x,y)
        self.arrow_sim.setRotation(-np.degrees(h))

        #Dift plot of different test paths
        global path_current
        x_coords = []
        y_coords = []
        x_fpd = []
        y_fpd = []

        for ps in path_current.poses:
            x_coords.append(ps.pose.position.x)
            y_coords.append(ps.pose.position.y)

            x_fpd.append(rx(ps.pose.position.x, ps.pose.position.y))
            y_fpd.append(ry(ps.pose.position.x, ps.pose.position.y))

        lcolor = 'w'
        if tags[0] > 0.1 or tags[1] > 0.1:
            lcolor = 'y'

        #if Psi[0] > 0.2 or Psi[1] > 0.2:
        #    lcolor = 'r'

        self.main_plot.setData(x_fpd, y_fpd,
                               clear = True,
                               pen = pg.mkPen(lcolor, width = 3))
        self.racetrack_plot.setData(x_coords, y_coords,
                               clear = True,
                               pen = pg.mkPen(lcolor, width = 3))

        # W wedge UPDATE
        uID = 1 # 2
        w_anrange_min = tag_ranges[uID][qID][anID][0]
        w_anrange_max = tag_ranges[uID][qID][anID][1]
        w_anrange_span = w_anrange_max - w_anrange_min
        w_anrange_start = w_anrange_min
        self.w_indicator_anwedge.setStartAngle(
            int(5760*(w_anrange_start-0.25)))
        self.w_indicator_anwedge.setSpanAngle(
            int(5760*w_anrange_span))

        w_range_min = tag_ranges[uID][qID][nID][0]
        w_range_max = tag_ranges[uID][qID][nID][1]
        w_range_span = w_range_max - w_range_min
        w_range_start = w_range_min
        self.w_indicator_wedge.setStartAngle(int(5760*(w_range_start-0.25)))
        self.w_indicator_wedge.setSpanAngle(int(5760*w_range_span))

        # v Needle UPDATE
        vnt = 0.02
        r = 0.3
        # alpha = angle (radians) measured like QgraphicsEllipseItem
        alpha = (v - 0.25)*2*np.pi
        # the needle tip will be the center of the ellipse
        # angle in QGE goes in opposite direction of mathematics
        vnx = r * np.cos(-alpha)
        vny = r * np.sin(-alpha)
        # QGE box goes translated below and left of center
        vnbox_x = vnx - r
        vnbox_y = vny - r
        # Updating box
        # center offset -2 + 0.1, -0.3
        self.v_needle.setRect(vnbox_x -2 + 0.1 + r, vnbox_y, r*2, r*2)
        self.v_needle.setStartAngle(int(5760*(v - vnt/2.0 +0.25)))
        self.v_needle.setSpanAngle(int(5760*vnt))

        # w Needle UPDATE
        wnt = 0.02
        r = 0.3
        # alpha = angle (radians) measured like QgraphicsEllipseItem
        alpha = (w - 0.25)*2*np.pi
        # the needle tip will be the center of the ellipse
        # angle in QGE goes in opposite direction of mathematics
        wnx = r * np.cos(-alpha)
        wny = r * np.sin(-alpha)
        # QGE box goes translated below and left of center
        wnbox_x = wnx - r
        wnbox_y = wny - r
        # Updating box
        # center offset -2 + 0.1, -0.3
        self.w_needle.setRect(wnbox_x -2 + 0.1 + xwn + r, wnbox_y, r*2, r*2)
        self.w_needle.setStartAngle(int(5760*(w - wnt/2.0 +0.25)))
        self.w_needle.setSpanAngle(int(5760*wnt))

        # Conflict Wedge UPDATE
        satgamma = usat(gamma)
        self.vc_wedge.setStartAngle(int(5760*(-0.25)))
        self.vc_wedge.setSpanAngle(int(5760*satgamma))
        self.vs_wedge.setStartAngle(int(5760*(-0.25 + satgamma)))
        tagspace = tags[0]
        if tags[0] > 1 - satgamma:
            tagspace = 1 - satgamma
        self.vs_wedge.setSpanAngle(int(5760*(tagspace)))
        self.va_wedge.setStartAngle(int(5760*(-0.25)))
        self.va_wedge.setSpanAngle(int(5760*Psi[0]))

        self.wc_wedge.setStartAngle(int(5760*(-0.25)))
        self.wc_wedge.setSpanAngle(int(5760*satgamma))
        self.ws_wedge.setStartAngle(int(5760*(-0.25 + satgamma)))
        tagspace = tags[1]
        if tags[1] > 1 - satgamma:
            tagspace = 1 - satgamma

        self.ws_wedge.setSpanAngle(int(5760*(tagspace)))
        self.wa_wedge.setStartAngle(int(5760*(-0.25)))
        self.wa_wedge.setSpanAngle(int(5760*Psi[1]))


        # RACETRACK ARC UPDATE
        garcpoints = [[None, None],[None, None]]

        p = 0
        garcpoints[p][0] = [rx( 2, 0) -1, ry( 2, 0) -1,  2, 2]
        garcpoints[p][1] = [rx(-2, 0) -1, ry(-2, 0) -1, 2, 2]

        for a in range(0, 2):
            self.garc[p][a].setRect(garcpoints[p][a][0],
                                    garcpoints[p][a][1],
                                    garcpoints[p][a][2],
                                    garcpoints[p][a][3])

        rotang = -(- h - np.pi*0.5)/(2*np.pi)


        self.garc[p][0].setStartAngle(int(5760*(rotang-0.25)))
        self.garc[p][1].setStartAngle(int(5760*(rotang-0.25)))
        self.garc[p][0].setSpanAngle(-int(5760* 0.5))
        self.garc[p][1].setSpanAngle(int(5760* 0.5))

        # Lines

        ln = 8
        linepoints = [[None]*ln, [None]*ln]

        linepoints[p][0] = [rx(-4, 2), ry(-4, 2), rx(4, 2), ry(4, 2)]
        linepoints[p][1] = [rx(4, 2),  ry(4, 2),  rx(4, -2), ry(4,-2)]
        linepoints[p][2] = [rx(4, -2), ry(4, -2), rx(-4, -2), ry(-4, -2)]
        linepoints[p][3] = [rx(-4,-2), ry(-4,-2), rx(-4, 2), ry(-4, 2)]
        linepoints[p][4] = [rx(-2, 1), ry(-2, 1), rx(2, 1), ry(2, 1)]
        linepoints[p][5] = [rx(-2, -1),ry(-2,-1), rx(2, -1), ry(2, -1)]
        linepoints[p][6] = [rx(-2, 1), ry(-2, 1), rx(-2, -1), ry(-2, -1)]
        linepoints[p][7] = [rx(2, 1),  ry(2, 1),  rx(2, -1), ry(2, -1)]

        for l in range(0, 8):
            self.lines[p][l].setLine(linepoints[p][l][0],
                                     linepoints[p][l][1],
                                     linepoints[p][l][2],
                                     linepoints[p][l][3])

        #Update the gui
        self.app.processEvents()

    def run(self):
        self.app.exec_()

GUI = GUI_haws()

#Start Qt Loop
if __name__ == '__main__':
    start()

