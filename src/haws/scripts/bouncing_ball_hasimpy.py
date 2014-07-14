# -*- coding: utf-8 -*-
"""
Created on Tue May 27 10:01:32 2014

@author: Rafael Figueroa
"""

#graphviz,D3
import os

import logging
import sys
logging.basicConfig(stream=sys.stderr,level=logging.DEBUG)
from hasimpy import *

def g0(X):
    if tolEqual(X[0],0) and X[1]<0:
        return True
    else:
        return False

def r0(X):
    X[0]= 0
    X[1]=-0.9*X[1]
    return X

e0=E([0],[g0],[r0])

def f0(X,t=0):
    'bouncing ball SS, LTI description'
    dX=np.array([0.0,0.0])
    dX[0]=X[1]
    dX[1]=-9.81
    #print ('dx1: %1.3f dx2: %1.3f' % (dX[0],dX[1]))
    return np.array(dX)

q0=Q(0,f0,e0)

h=H([q0],2)

X0=np.array([1,0])
t0=0
tlim=6
print 'simulating:'
simResult = h.sim(0,X0,t0,tlim)
simResult.simPlot()

