#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

# python imports
import numpy as np
import math

# haws imports
from ha_model import *
import ha_model
import dift_model

class DiftModel(object):
    """L(uID, qID) = npdfs matrix
    npdfs = normalized pdfs
    pdfs = probability density functions"""

    def __init__(self, L, un_md):
        self.L = L
        self.un_md = un_md # nominal input mean and deviation

    def create_tags(self, qID, input):
        tags = [None]*len(input)
        for uID, u in enumerate(input):
            tags[uID] = 1.0 - self.L[uID][qID](u)

        return tags

    def tag_ranges(self):
        # todo: take from ha
        inputs_number = 2
        modes_number = 2
        tr = [[None, None], [None, None]]
        low = 0.1
        med = 0.3

        for uID in range(0, inputs_number):
            for qID in range(0, modes_number):

                u_mean = self.un_md[uID][qID][0]
                u_devsqrt = np.sqrt(self.un_md[uID][qID][1])

                tr[uID][qID] = [[u_mean - u_devsqrt * low,
                                 u_mean + u_devsqrt * low],
                                [u_mean - u_devsqrt * med,
                                 u_mean + u_devsqrt * med]]

        return tr



