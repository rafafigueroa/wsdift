#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
print " *** running racetrack stand alone *** "

from ha_model import *
import ha_model
from hasimpy import *

#initial state
X0=np.array([0.0,0.5,0.0])
STRAIGHT = 0 #qID
CURVE = 1 #qID

t0 = 0
tlim = 10
print 'simulating:'
# with initial conditions
qID0 = STRAIGHT
simResult = h.sim(qID0, X0, h.q[qID0].u, t0, tlim,
                  debug_flag = False)
simResult.phasePlot([0,1])
simResult.simPlot()

raw_input('\n Press ENTER to finish program and close plots')

