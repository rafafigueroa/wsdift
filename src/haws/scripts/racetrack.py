#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
print " *** running racetrack setup *** "
dp = True
import numpy as np
sin = np.sin
cos = np.cos
pi = np.pi
#from scipy.integrate import odeint

from inspect import isfunction

#logging for debug
import logging,sys
logging.basicConfig(stream=sys.stderr,level=logging.DEBUG)

from hasimpy import *

TRACK_LIMIT_X = 2

def u0(X):
    'straight control'
    return np.array([2.0,0.0])

def f0(X,u0,t=0):
    'Straight Path'
    if isfunction(u0):
        u=u0(X)
    else:
        u=u0

    v=u[0]
    w=u[1] #omega

    x=X[0]
    y=X[1]
    h=X[2] #theta

    dx=v*cos(h)
    dy=v*sin(h)
    dh=w

    dX=np.array([dx,dy,dh])
    return np.array(dX)

def u1(X):
    'curve control'
    return np.array([2.0,-20*pi/60.0])
    
def f1(X,u1,t=0):
    'Curved path'
    if isfunction(u1):
        u=u1(X)
    else:
        u=u1

    v=u[0]
    w=u[1] #omega

    x=X[0]
    y=X[1]
    h=X[2] #theta

    dx=v*cos(h)
    dy=v*sin(h)
    dh=w

    dX=np.array([dx,dy,dh])
    return np.array(dX)
    
def g0(X):
    'Start curve'
    x=X[0]
    y=X[1]
    h=X[2] #theta

    return (x>TRACK_LIMIT_X and y>0) or (x<-TRACK_LIMIT_X and y<0)

def g1(X):
    'Start straight'
    x=X[0]
    y=X[1]
    h=X[2] #theta

    return (x>-TRACK_LIMIT_X and x<0 and y>0) or \
        (x<TRACK_LIMIT_X and x>0 and y<0)

def avoid(X):
    'True when inside the avoid set'
    x=X[0]
    y=X[1]
    h=X[2] #theta

    return not (x>1 and x<10 and y>1 and y<10)
    
def avoid_2(X):
    'True when inside the avoid set, for use when running pure python'
    x=X[0]
    y=X[1]
    h=X[2] #theta

    return (x>5 or x<-5 or y>5 or y<-5)
    
#Identity Reset Maps
#edges
e0=E([1],[g0],[idem])
e1=E([0],[g1],[idem])

#discrete states
q0=Q(0,f0,u0,e0,Dom=any,Avoid=avoid) #straight
q1=Q(1,f1,u1,e1,Dom=any,Avoid=avoid) #curve

#hybrid automata (continuous dynamics)
h=H([q0,q1],1)

#change to True when running it by itself
if False:
    #initial state
    X0=np.array([0.0,2.0,0.0])
    STRAIGHT = 0 #qID
    CURVE = 1 #qID

    t0=0
    tlim=15
    print 'simulating:'
    # with initial conditions
    qID0 = STRAIGHT
    simResult = h.sim(qID0,X0,h.q[qID0].u,t0,tlim,debug_flag = True)
    simResult.phasePlot([0,1])
    simResult.simPlot()

    raw_input('\n Press ENTER to finish program and close plots')

