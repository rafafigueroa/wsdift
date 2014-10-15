#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

#Python imports
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

#pyqtgraph configuration
pg.setConfigOptions(antialias = True)


class GUI_haws:
    def __init__(self):
        #Qt Initialization (once per application)
        self.app = QtGui.QApplication([])

        #Top level Qt Widget
        self.mw = QtGui.QWidget()

        #Plot widgets and objects
        sim_plot_widget = pg.PlotWidget(name = 'sim')
        self.sim_plot = sim_plot_widget.plot(title = 'Robot Simulation')

        haws_plot_widget = pg.PlotWidget(name = 'haws')
        self.haws_plot = haws_plot_widget.plot(title = 'Information Tracking')

        #Layout
        layout = QtGui.QGridLayout()
        layout.addWidget(sim_plot_widget,0,0)
        layout.addWidget(haws_plot_widget,0,1)
        self.mw.setLayout(layout)
        #self.mw.show()

        #GUI Loop timer

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(50.0) #50 ms


    def update_gui(self):
        self.sim_plot.setData([0,1],[0,-1])
        self.haws_plot.setData([0,1],[0,1])
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
#if __name__ == '__main__':

GUI = GUI_haws()
GUI.run()
