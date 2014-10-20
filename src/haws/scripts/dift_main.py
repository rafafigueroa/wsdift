#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

# python imports
import numpy as np

# haws imports
from ha_model import *
import ha_model
import dift_model

class DiftModel(object):
    # L(uID, qID) = npdfs matrix
    # npdfs = normalized pdfs
    # pdfs = probability density functions

    def __init__(self, L):
        self.L = L

    def create_tags(self, qID, input):
        tags = [None]*len(input)
        for uID, u in enumerate(input):
            tags[uID] = 1.0 - self.L[uID][qID](u)

        return tags
