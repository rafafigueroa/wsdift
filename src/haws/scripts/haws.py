#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  7 15:47:03 2014

@author: Rafael Figueroa

"""


def input_create_options(u):
    #TODO: make general for any number of inputs
    #hint: use tuples/
    #or something like positions = [(i,j) for i in range(5) for j in range(4)]
    
    v = u[0]
    w = u[1]
    
    p1 = [0, 0]
    p2 = [v, 0]
    p3 = [0, w]
    p4 = [v, w]
    
    return [p1, p2, p3, p4]


def tags(time_to_avoid, avoid_activated, ALERT):
    #TODO: make general
    #TODO fix number with ALERT
    
    p_tags = [0]*len(avoid_activated)
    
    for pID, pOption in enumerate(time_to_avoid):
        if avoid_activated[pID]:
            p_tags[pID] = ALERT/(pOption+ALERT/10.0)
        else:
            p_tags[pID] = 0
    
    do_nothing_tags = p_tags[0]
    current_tags = p_tags[3]
    first_input_tag = p_tags[1]
    second_input_tag = p_tags[2]
    
    v_tag = current_tags*(first_input_tag/(second_input_tag+ALERT/10.0)) - \
        do_nothing_tags
    w_tag = current_tags*(second_input_tag/(first_input_tag+ALERT/10.0)) - \
        do_nothing_tags
    
    print '\n------------ TAGS --------------'
    print 'v tags', v_tag
    print 'w tags', w_tag
    print '--------------------------------\n'
    
    return [v_tag, w_tag]


