#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May 30 16:15:23 2014

@author: Rafael Figueroa
"""


import logging
import sys
logging.basicConfig(stream=sys.stderr,level=logging.DEBUG)
from hasimpy import *

def g0(X):
    return X[0]<=19

def g1(X):
    return X[0]>=21

#Identity Reset Maps
e0=E([1],[g0],[idem])
e1=E([0],[g1],[idem])

def f0(X,t=0):
    'Heater OFF'
    dX=np.array([0.0])
    dX[0]=np.array(-0.1*X[0])
    return np.array(dX)

def f1(X,t=0):
    'Heater ON'
    dX=np.array([0.0])
    dX[0]=np.array(-0.1*(X[0]-30))
    return np.array(dX)

def dom0(X,t=0):
    'Heater OFF'
    return X>19

def dom1(X,t=0):
    'Heater ON'
    return X<21


q0=Q(0,f0,e0,Dom=dom0)
q1=Q(1,f1,e1,Dom=dom1)

h=H([q0,q1],1)

X0=np.array([24])
HEATER_OFF = 0 #for starting temp > 19
HEATER_ON = 1 #for starting temp < 21

t0=0
tlim=10
print 'simulating:'
# with initial conditions
simResult = h.sim(HEATER_OFF,X0,t0,tlim)
simResult.simPlot()



