#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
from __future__ import division

from pylab import *
from math import *
from matplotlib.pyplot import *
from scipy import *
from scipy.integrate import odeint
from pylab import *
import numpy as np
import itertools
from collections import deque

close('all')
TEST_INPUT = "circle"

# READ DATA FROM FILES
experiment_file = 'log_circle_test_input.csv'
fptr = open('/home/rafa/Dropbox/aDift/logs/' + \
            experiment_file, 'r')
file_text = fptr.readlines()

kmax = len(file_text)
t = [None]*kmax
qID = [None]*kmax
x = [None]*kmax
y = [None]*kmax
yaw = [None]*kmax
vx = [None]*kmax
wz = [None]*kmax
tc = [None]*kmax
tagv = [None]*kmax
tagw = [None]*kmax
wlv = [None]*kmax
wlw = [None]*kmax

for i, line in enumerate(file_text):
    lines_values = line.split(',')
    t[i] = float(lines_values[0])
    qID[i] = int(lines_values[1])
    x[i] = float(lines_values[2])
    y[i] = float(lines_values[3])
    yaw[i] = float(lines_values[4])
    vx[i] = float(lines_values[5])
    wz[i] = float(lines_values[6])
    tc[i] = float(lines_values[7])
    tagv[i] = float(lines_values[8])
    tagw[i] = float(lines_values[9])
    wlv[i] = float(lines_values[10])
    wlw[i] = float(lines_values[11])

nnbias = [1]*kmax
nnu = np.array([nnbias, tagv, tc])
# Desired Values
dv = [0.0]*kmax
dw = [0.0]*kmax


# find t1, t2
t1_found = False
t2_found = False
t1 = None
t2 = None
k1 = None
k2 = None

LOW = 0.3
ALERT = 1.0

for k in range(0,kmax):
    if x[k] > 0 and not t1_found:
        t1_found = True
        t1 = t[k]
        k1 = k

    if t1_found and not t2_found:
        if t[k] - t1 > ALERT:
            t2 = t[k]
            t2_found = True

print 't1', t1, 't2', t2

for k in range(0,kmax):
    dv[k] = 0
    if t[k] < t1:
        dw[k] = 0
    elif t[k] >= t1 and t[k] < t2:
        dw[k] = LOW*(t[k] -t1)/ALERT
    elif t[k] >= t2:
        dw[k] = LOW


# Learning Algorithm
nnW = np.array([-0.5, 0.0, 0.1, 0.1])
nnW = np.transpose(nnW)
nnxe = np.array([0.0, 1.0, 0.0, 0.0])
nnxe = np.transpose(nnxe)
nnL = np.array([0.0]*4)
nnL = np.transpose(nnL)
eta0 = 0.3
eta = eta0
eta_changes = 0

ALERT = 1.0

maxepochs = 10000
first_error = True
previous_error = 0

for epoch_number in range(0, maxepochs):
    E = 0.0
    DW = 0.0

    for k in range(0, kmax):
        nnxe[2] = ALERT/tc[k]
        nnxe[3] = tagw[k]
        v = np.dot(np.transpose(nnW), nnxe)
        phi = 1/(np.exp(5 - 10*v) + 1)
        phip = (10*np.exp(5 - 10*v))/(np.exp(5 - 10*v) + 1)**2
        e = dw[k] - phi
        E += e * e
        DW += eta*nnL*e
        nnL = phip*(nnxe[0]*nnL + nnxe)
        #nnW += DW
        nnxe[0] = phi

    if first_error:
        previous_error = np.sqrt(E/kmax)
        first_error = False

    if previous_error <= np.sqrt(E/kmax):
        eta_changes += 1
        eta = eta0 * np.exp(- eta_changes/1000.0)
    else:
        previous_error = np.sqrt(E/kmax)

    nnW += DW/kmax

    print epoch_number,': ', eta, ':', np.sqrt(E/kmax)

# After Learning
nny = [None]*kmax
nn_gamma = [None]*kmax
nn_varsigma = [None]*kmax
nnxe = [0.0, 1.0, 0.0, 0.0]
for k in range(0, kmax):
    nnxe[2] = ALERT/tc[k]
    nn_gamma[k] = ALERT/tc[k]
    nn_varsigma[k] = tagw[k]
    nnxe[3] = tagw[k]
    v = np.dot(np.transpose(nnW), nnxe)
    phi = 1/(np.exp(5 - 10*v) + 1)
    phip = (10*np.exp(5 - 10*v))/(np.exp(5 - 10*v) + 1)**2
    e = dw[k] - phi
    nnL = phip*(nnxe[0]*nnL + nnxe)
    nnxe[0] = phi
    nny[k] = phi

#print nny
print 'W', nnW

fig = figure()

plot(t, dw, label='Desired')
hold('on')
plot(t, nny, label='Learned')
plot(t, nn_gamma, label='Conflict')
plot(t, nn_varsigma, label='Dift tag')
legend()
ax = fig.add_subplot(111)
xlabel('Time steps')
ylabel('Warning Level')
grid(True)
plt.savefig('w_desired_vs_trained' + '.eps',
            format='eps', dpi=1000)

plt.show()


