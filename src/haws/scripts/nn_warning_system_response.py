#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
from __future__ import division
import haws_variables

TEST_INPUT = haws_variables.test_input
LOW = haws_variables.low
MED = haws_variables.med
HIGH = haws_variables.high
ALERT = haws_variables.alert
MINPHI = haws_variables.minphi

from pylab import *
from math import *
from matplotlib.pyplot import *
from scipy import *
from scipy.integrate import odeint
from pylab import *
import numpy as np
import itertools
from collections import deque
from nn_ddrn import *

def dfw_circle(x, y, yaw, t):
    kmax = len(x)
    dw = [None]*kmax
    # find t1, t2
    t1_found = False
    t2_found = False
    t3_found = False
    t4_found = False
    t1 = None
    t2 = None
    t3 = None
    t4 = None

    tss = None
    tse = None
    tcs = None
    tce = None

    for k in range(0, kmax):
        if x[k] > 0 and not t1_found:
            t1_found = True
            t1 = t[k]
            tss = k

        if t1_found and not t2_found:
            if t[k] - t1 > ALERT:
                t2 = t[k]
                t2_found = True

        if t1_found and t2_found and not t3_found:
            if x[k] < 0:
                t3 = t[k]
                t3_found = True
                tse = k

        if t1_found and t2_found and t3_found and not t4_found:
            if t[k] - t3 > ALERT:
                t4 = t[k]
                t4_found = True

    #print 't1', t1, 't2', t2

    LOW = 0.1
    for k in range(0,kmax):
        if t[k] < t1:
            dw[k] = MINPHI
        elif t[k] >= t1 and t[k] < t2:
            dw[k] = LOW*(t[k] -t1)/ALERT
        elif t[k] >= t2 and t[k] < t3:
            dw[k] = LOW
        elif t[k] >= t3 and t[k] < t4:
            # down from LOW
            dw[k] = LOW  - (LOW/ALERT)*(t[k] - t3)
        elif t[k] >= t4:
            dw[k] = MINPHI

    tkey = [tss, tse, tcs, tce]

    return [dw, tkey]

def dfw_diag(x, y, yaw, t):
    kmax = len(x)
    dw = [None]*kmax

    tss = None
    tse = None
    tcs = None
    tce = None

    for k in range(0, kmax):

        if t[k] == 1.44:
            tcs = k

        if t[k] == 5.82:
            tss = k
            tce = k

        if t[k] == 6.92:
            tse = k

    t1 = 1.44
    t2 = 5.82
    t3 = 6.92

    MED = 0.5

    for k in range(0,kmax):
        if t[k] < t1:
            dw[k] = MINPHI
        elif t[k] >= t1 and t[k] < t2:
            dw[k] = MINPHI + (t[k] - t1)*(HIGH-MINPHI)/(t2-t1)
        elif t[k] >= t2 and t[k] < t3:
            dw[k] = MED
        elif t[k] >= t3 and t[k] < (t3+ALERT):
            dw[k] = MED + (t[k] - t3)*(MINPHI-MED)/ALERT
        elif t[k] >= (t3 + ALERT):
            dw[k] = MINPHI

    tkey = [tss, tse, tcs, tce]

    return [dw, tkey]

def dfw_bump(x, y, yaw, t):
    kmax = len(x)
    dw = [None]*kmax
    # find t1, t2

    t1_found = False
    t2_found = False
    t3_found = False
    t4_found = False
    t5_found = False
    t6_found = False
    t7_found = False
    t8_found = False

    t1 = None
    t2 = None
    t3 = None
    t4 = None
    t5 = None
    t6 = None
    t7 = None
    t8 = None

    tss = None
    tse = None
    tcs = None
    tce = None

    for k in range(0, kmax):
        'from experiment log'

        if t[k] == 4.38:
            tss = k

        if t[k] == 6.40:
            tcs = k

        if t[k] == 9.94:
            tce = k

        if t[k] == 18.66:
            tse = k

    for k in range(0, kmax):

        if y[k] > 1.0 and not t1_found:
            t1_found = True
            t1 = t[k]

        if t1_found and not t2_found:
            if t[k] - t1 > ALERT:
                t2 = t[k]
                t2_found = True

        if t1_found and t2_found and not t3_found:
            'from current log'
            if t[k] == 6.40:
                t3 = t[k]
                t3_found = True

        if t1_found and t2_found and t3_found \
                and not t4_found:
            if t[k] == 9.94:
                t4 = t[k]
                t4_found = True

        if t1_found and t2_found and t3_found \
                and t4_found and not t5_found:
            if y[k] < 1.7:
                t5 = t[k]
                t5_found = True

        if t1_found and t2_found and t3_found \
                and t4_found and t5_found \
                and not t6_found:
            if t[k] - t5 > ALERT:
                t6 = t[k]
                t6_found = True

        if t1_found and t2_found and t3_found \
                and t4_found and t5_found \
                and t6_found and not t7_found:
            if x[k] > 0.9:
                t7 = t[k]
                t7_found = True

        if t1_found and t2_found and t3_found \
                and t4_found and t5_found \
                and t6_found and t7_found \
                and not t8_found:
            if t[k] - t7 > ALERT:
                t8 = t[k]
                t8_found = True

    print t1, t2, t3, t4, t5, t6, t7, t8

    LOW = 0.15
    MED = 0.5

    for k in range(0,kmax):
        if t[k] < t1:
            dw[k] = MINPHI
        elif t[k] >= t1 and t[k] < t2:
            dw[k] = MINPHI + (t[k] - t1)*(LOW-MINPHI)/ALERT
        elif t[k] >= t2 and t[k] < t3:
            dw[k] = LOW
        elif t[k] >= t3 and t[k] < t4:
            dw[k] =  LOW + (t[k] - t3)*(MED-LOW)/(t4-t3)
        elif t[k] >= t4 and t[k] < t5:
            dw[k] = MED
        elif t[k] >= t5 and t[k] < t6:
            dw[k] =  MED + (t[k] - t5)*(LOW-MED)/ALERT
        elif t[k] >= t6 and t[k] < t7:
            dw[k] = LOW
        elif t[k] >= t7 and t[k] < t8:
            dw[k] =  LOW + (t[k] - t7)*(MINPHI-LOW)/ALERT
        elif t[k] >= t8:
            dw[k] = MINPHI

    tkey = [tss, tse, tcs, tce]

    return [dw, tkey]

# ************** Object creation ***************
#VW0 = np.array([-0.1, 0.01, 0.4, 0.4])
# -0.39968515  0.00957328  0.20221609  0.2057081
# np.array([-0.35277376, 0.31462329, 0.27082417, 0.40929885])
# [-0.12279497  0.31444523  0.27082417  0.40929885]
#W0 = np.array([-0.25277376, 0.31462329, 0.27082417, 0.40929885])
# [-0.15735388  0.29804371  0.27082417  0.40929885]
# [-0.05555376, 0.01462329, 0.27082417, 0.40929885]
# np.array([-0.15735388, 0.09804371, 0.27082417, 0.40929885])

W0 = np.array([-0.15735388, 0.09804371, 0.27082417, 0.40929885])
# np.array([-0.5, 0.06, 0.5, 0.9])
Wcv = np.array(W0)
Wcw = np.array(W0) # [0.47983289, 0.05575454, 0.4, 1.18927859]
Wbw = np.array(W0)
Wdw = np.array(W0)
alarm_levels = [LOW, MED, HIGH]

'circle experiment'
#nn_cv = RecurrentNeuronExperiment('circle', 'v', 'log_circle_test_input.csv',
#                                      Wcv, dfv_circle,  alarm_levels, ALERT)

nn_cw = RecurrentNeuronExperiment(
    'circle', 'w', 'log_experiment_circle.csv',
    Wcw, dfw_circle, alarm_levels, ALERT)

nn_bw = RecurrentNeuronExperiment(
    'bump', 'w', 'log_experiment_bump.csv',
    Wbw, dfw_bump, alarm_levels, ALERT)

nn_dw = RecurrentNeuronExperiment(
    'diag', 'w', 'log_experiment_diag.csv',
    Wdw, dfw_diag, alarm_levels, ALERT)


nn_exp_list = [nn_dw, nn_bw, nn_cw]
nn_DW_list = [None, None, None]
#
# # calculate DW for each case separately
for nni, nn in enumerate(nn_exp_list):

    for p in range(0, 2000):

        DW_w = nn.presentation()
        nn.W += DW_w

    print 'W:', nn.W
    DW_total = W0 - nn.W
    nn_DW_list[nni] = DW_total

print 'DW List:'
print nn_DW_list
print '****** calculating means ******'
# calcuate the mean of the results
DW_total = np.array([0.0, 0.0, 0.0, 0.0])
for DW in nn_DW_list:
    DW_total += DW
DW_mean = np.array(DW_total/3.0)
print 'DW mean:', DW_mean

for nn in nn_exp_list:
    nn.W = np.array(W0 + DW_mean)

for p in range(0, 1000):

    DW_w = np.array([0.0, 0.0, 0.0, 0.0])
    for nn in nn_exp_list:
        DW_w += nn.presentation()

    for nn in nn_exp_list:
        nn.W += DW_w

for nn in nn_exp_list:
   nn.plot_response()

for nn in nn_exp_list:
    nn.save_plots()
