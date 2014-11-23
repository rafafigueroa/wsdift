#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
from __future__ import division
import numpy as np

test_input = 'circle'
x0 = 0
y0 = 0
h0 = 0

if test_input == 'circle':
    'spurious high, conflict low'
    x0 = -1.0
    y0 = 1.0
    h0 = 0.0
elif test_input == 'line':
    x0 = 1.0
    y0 = 1.0
    h0 = 0.0
elif test_input == 'bump':
    x0 = -2.0
    y0 = 1.0
    h0 = 0.0
elif test_input == 'diag':
    x0 = -1.0
    y0 = 1.0
    h0 = np.arctan2(1.0, 1.0)

low = 0.3
med = 0.6
high = 0.8
alert = 1.0
minphi = 0.00669285
Ts = 0.02