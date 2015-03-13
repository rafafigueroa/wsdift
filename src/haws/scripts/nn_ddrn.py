#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
from __future__ import division

from pylab import *
from math import *
from matplotlib.pyplot import *
from matplotlib.patches import Ellipse, Arc
from scipy import *
from scipy.integrate import odeint
from pylab import *
import numpy as np
import itertools
from collections import deque

class RecurrentNeuronExperiment(object):
    def __init__(self, exp_name, tagvar, fname, W,
                 desired_function_times,
                 alarm_levels, alert):

        self.exp_name = exp_name
        self.tagvar = tagvar
        self.W = W
        self.bestW = np.array(W)
        self.ALERT = alert
        self.LOW = alarm_levels[0]
        self.MED = alarm_levels[1]
        self.HIGH = alarm_levels[2]

        self.keeplearning = True
        self.eta_change_count = 0

        # FROM FILES TO VARIABLES

        experiment_file = fname
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

        for i, line in enumerate(file_text):
            lines_values = line.split(',')
            t[i] = float(lines_values[0])
            qID[i] = int(lines_values[1])
            x[i] = float(lines_values[2])
            y[i] = float(lines_values[3])
            yaw[i] = float(lines_values[4])
            vx[i] = float(lines_values[5])
            wz[i] = float(lines_values[6])
            if lines_values[7] == 'inf':
                tc[i] = 10000.0
            else:
                tc[i] = float(lines_values[7])
            tagv[i] = float(lines_values[8])
            tagw[i] = float(lines_values[9])

        self.kmax = kmax
        self.t = t
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.wz = wz
        self.tc = tc

        if self.tagvar == 'v':
            self.tag = tagv
        elif self.tagvar == 'w':
            self.tag = tagw

        # DESIRED VALUES

        dft = desired_function_times(x, y, yaw, t)
        self.d = dft[0]
        self.keyt = dft[1]

        # Learning Algorithm
        self.W = np.transpose(self.W)

        self.eta0 = 0.05
        self.eta = self.eta0
        self.eta_changes = 0
        self.first_error = True
        self.previous_error = 0

        self.nnxe = np.array([0.0, 1.0, 0.0, 0.0])
        self.nnxe = np.transpose(self.nnxe)
        self.nnL = np.array([0.0]*4)
        self.nnL = np.transpose(self.nnL)

    def presentation(self):

        d = self.d
        tag = self.tag
        ALERT = self.ALERT
        kmax = self.kmax
        tc = self.tc
        # System stores one nnxe

        DW = np.array([0.0, 0.0, 0.0, 0.0])
        E = 0.0

        if np.isnan(self.W[0]):
            wait = input('W is nan')

        if self.keeplearning:
            for k in range(0, kmax):
                if tc[k] == np.inf:
                    self.nnxe[2] = 0.0
                elif tc[k] == 0.0:
                    self.nnxe[2] = 10.0
                else:
                    self.nnxe[2] = ALERT/float(tc[k])
                self.nnxe[3] = tag[k]
                v = np.dot(np.transpose(self.W), self.nnxe)
                #print self.nnxe
                #print self.W
                #print v
                phi = 1/(np.exp(5 - 10*v) + 1.0)
                if np.isnan(phi):
                    print 'nnxe:', self.nnxe
                    print 'tag:', tag[k],
                    print 'alert:', ALERT/float(tc[k])
                    print 'W:', self.W
                    print 'v:', v
                    print 'phi:', phi
                    wait = input("PRESS ENTER TO CONTINUE.")

                phip = (10*np.exp(5 - 10*v))/(np.exp(5 - 10*v) + 1)**2
                e = d[k] - phi
                E += e * e
                DW += self.eta*self.nnL*e
                self.nnL = phip*(self.nnxe[0]*self.nnL + \
                                 self.nnxe)
                #nnW += DW
                self.nnxe[0] = phi

                if np.isnan(phi):
                    print 'nnxe:', self.nnxe
                    print 'tag:', tag[k],
                    print 'alert:', ALERT/float(tc[k])
                    print 'W:', self.W
                    print 'v:', v
                    print 'phi:', phi
                    wait = input("PRESS ENTER TO CONTINUE.")

            if self.first_error:
                self.previous_error = np.sqrt(E/kmax)
                self.first_error = False

            if self.previous_error <= np.sqrt(E/kmax):
                self.eta_changes += 1
                self.eta = self.eta0 * np.exp(- self.eta_changes/200.0)
                self.W = np.array(self.bestW)
                #print self.exp_name, self.eta
                if self.eta_changes == 200:
                    self.keeplearning = False
                    print 'stopped learning'

            else:
                self.previous_error = np.sqrt(E/kmax)
                self.bestW = np.array(self.W)
                # print self.previous_error, self.W

        return DW/kmax

    def plot_response(self):

        ALERT = self.ALERT
        kmax = self.kmax
        tc = self.tc
        d = self.d
        tag = self.tag
        tagvar = self.tagvar

        nny = [None]*kmax
        nnxe = [0.0, 1.0, 0.0, 0.0]
        nn_gamma = [None]*kmax
        nn_varsigma = [None]*kmax

        print 'inside W:'
        print self.bestW

        for k in range(0, kmax):
            nnxe[2] = ALERT/float(tc[k])
            nnxe[3] = tag[k]
            nn_varsigma[k] = tag[k]
            nn_gamma[k] = ALERT/float(tc[k])
            v = np.dot(np.transpose(self.bestW), nnxe)
            phi = 1/(np.exp(5 - 10*v) + 1.0)
            nnxe[0] = phi
            nny[k] = phi

        fig = figure()

        # print nny

        t = self.t
        plot(t, d, label='Desired')
        hold('on')
        plot(t, nny, label='Learned')
        plot(t, nn_gamma, label='Conflict')
        plot(t, nn_varsigma, label='Dift tag')
        legend()
        ax = fig.add_subplot(111)
        xlabel('Time steps')
        ylabel('Warning Level')
        grid(True)
        plt.savefig('w_desired_vs_trained_' +
                    self.exp_name +
                    '_' + tagvar +'.eps',
                    format='eps', dpi=1000)

        plt.show()


    def save_plots(self):

        ALERT = self.ALERT
        kmax = self.kmax
        tc = self.tc
        d = self.d
        tag = self.tag
        tagvar = self.tagvar
        figdir = '/home/rafa/dift/thesis_latex/figures/'

        nny = [None]*kmax
        nnxe = [0.0, 1.0, 0.0, 0.0]
        nn_gamma = [None]*kmax
        nn_varsigma = [None]*kmax

        print self.bestW
        print 'inside W:'

        for k in range(0, kmax):
            nnxe[2] = ALERT/float(tc[k])
            nnxe[3] = tag[k]
            nn_varsigma[k] = tag[k]
            nn_gamma[k] = ALERT/float(tc[k])
            v = np.dot(np.transpose(self.bestW), nnxe)
            phi = 1/(np.exp(5 - 10*v) + 1.0)
            nnxe[0] = phi
            nny[k] = phi

        # Control theory version
        # warning system starts at 0
        z1 = 0.0
        z2 = 0.0
        Ts = 0.02
        zp = [None]*kmax
        for k in range(0, kmax):
            u = 0.9*nn_gamma[k] + 0.5*nn_varsigma[k]
            dz1 = z2
            dz2 = -25*z1 -8*z2 + 24*u
            z1 = z1 + dz1*Ts
            z2 = z2 + dz2*Ts
            if z1 >= 0 or z1 <= 1:
                zp[k] = z1
            elif z1 > 1:
                zp[k] = 1.0
            elif z1 < 0:
                zp[k] = 0.0


        t = self.t
        x = self.x
        y = self.y

        ssc = 80 # small scatter circle
        msc = 120 # medium scatter circle
        plw = 2 # plot line weight

        # Path
        fig1 = figure()

        plot(x, y, label='Path', linewidth=plw, color='k')
        hold('on')
        plot([-2, 2], [1, 1], label='racetrack', color='g', linestyle='--')
        plot([-2, 2], [-1, -1], color='g', linestyle='--')
        plot([-4, 4], [2, 2], label='avoid boundary',
             color='r', linestyle='--')
        plot([-4, 4], [-2,-2], color='r', linestyle='--')
        plot([-4, -4], [-2, 2], color='r', linestyle='--')
        plot([4, 4], [-2, 2], color='r', linestyle='--')
        rightarch = Arc([2,0], 2, 2, angle=0, theta1=-90, theta2=90,
                        color='g', linestyle='dashed')
        leftarch = Arc([-2,0], 2, 2, angle=180, theta1=-90, theta2=90,
                        color='g', linestyle='dashed')

        ktss = self.keyt[0]
        ktse = self.keyt[1]
        ktcs = self.keyt[2]
        ktce = self.keyt[3]

        scatter(x[ktss], y[ktss], label='tss', s=ssc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.20])
        scatter(x[ktse], y[ktse], label='tse', s=ssc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.20])
        if ktcs is not None:
            scatter(x[ktcs], y[ktcs], label='tcs', s=msc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.80])
            scatter(x[ktce], y[ktce], label='tce', s=msc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.80])

        legend(loc='lower right')

        axis('equal')
        ax = fig1.add_subplot(111)
        ax.add_patch(rightarch)
        ax.add_patch(leftarch)
        ax.set_xlim([-4.5, 4.5])
        xlabel('x (m)')
        ylabel('y (m)')
        # grid(True)

        plt.savefig(figdir + 'path_' +
                    self.exp_name +
                    '_' + tagvar +'.eps',
                    format='eps', dpi=1000)

        hold('off')

        # omega
        omega = self.wz
        fig2 = figure()
        rc('text', usetex=True)

        plot(t, omega, label= r'$\omega$', linewidth=plw)
        hold('on')
        scatter(t[ktss], omega[ktss], label='tss', s=ssc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.20])
        scatter(t[ktse], omega[ktse], label='tse', s=ssc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.20])

        if ktcs is not None:
            scatter(t[ktcs], omega[ktcs], label='tcs', s=msc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.80])
            scatter(t[ktce], omega[ktce], label='tce', s=msc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.80])

        legend()
        ax = fig2.add_subplot(111)
        ax.set_ylim([-1.5,1.5])
        xlabel('Time (s)')
        ylabel(r'$\omega$')
        # grid(True)
        plt.savefig(figdir + 'omega_' +
                    self.exp_name +
                    '_' + tagvar +'.eps',
                    format='eps', dpi=1000)


        # desired vs learned
        fig3 = figure()
        rc('text', usetex=True)

        plot(t, d, label= 'desired', linestyle = 'dashed', linewidth=plw)
        hold('on')
        plot(t, nny, label='Learned', linewidth=plw)

        scatter(t[ktss], d[ktss], label='tss', s=ssc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.20])
        scatter(t[ktse], d[ktse], label='tse', s=ssc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.20])

        if ktcs is not None:
            scatter(t[ktcs], d[ktcs], label='tcs', s=msc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.80])
            scatter(t[ktce], d[ktce], label='tce', s=msc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.80])

        legend()
        ax = fig3.add_subplot(111)
        ax.set_ylim([-0.1,2.0])
        xlabel('Time (s)')
        ylabel('Warning level')
        # grid(True)
        plt.savefig(figdir + 'desired_vs_learned_' +
                    self.exp_name +
                    '_' + tagvar +'.eps',
                    format='eps', dpi=1000)

        # dift and conflict levels
        fig4 = figure()
        rc('text', usetex=True)
        hold('on')
        plot(t, nn_gamma, label='Conflict level', linewidth=plw,
             color= [0.8, 0.01, 0.10])
        plot(t, nn_varsigma, label='Dift tag', linewidth=plw,
            linestyle='dashed', color = [0.8, 0.01, 0.90])

        scatter(t[ktss], nn_varsigma[ktss], label='tss', s=ssc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.20])
        scatter(t[ktse], nn_varsigma[ktse], label='tse', 
                s=ssc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.20])

        if ktcs is not None:
            scatter(t[ktcs], nn_gamma[ktcs], label='tcs', 
                    s=msc, facecolors='none',
                    edgecolor = [0.8, 0.01, 0.80])

            scatter(t[ktce], nn_gamma[ktce], label='tce', 
                    s=msc, facecolors='none',
                    edgecolor = [0.01, 0.8, 0.80])

        legend()
        ax = fig4.add_subplot(111)
        ax.set_ylim([-0.1,2.0])
        xlabel('Time (s)')
        ylabel('Warning level')
        # grid(True)
        plt.savefig(figdir + 'dift_conflict_levels_' +
                    self.exp_name +
                    '_' + tagvar +'.eps',
                    format='eps', dpi=1000)

        # control theory
        fig5 = figure()
        rc('text', usetex=True)

        plot(t, d, label= 'desired', linestyle = 'dashed', linewidth=plw)
        hold('on')
        plot(t, zp, label='second order response', linewidth=plw)

        scatter(t[ktss], d[ktss], label='tss', s=ssc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.20])
        scatter(t[ktse], d[ktse], label='tse', s=ssc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.20])

        if ktcs is not None:
            scatter(t[ktcs], d[ktcs], label='tcs', s=msc, facecolors='none',
                edgecolor = [0.8, 0.01, 0.80])
            scatter(t[ktce], d[ktce], label='tce', s=msc, facecolors='none',
                edgecolor = [0.01, 0.8, 0.80])

        legend()
        ax = fig5.add_subplot(111)
        ax.set_ylim([-0.1,2.0])
        xlabel('Time (s)')
        ylabel('Warning level')
        # grid(True)
        plt.savefig(figdir + 'control_theory_' +
                    self.exp_name +
                    '_' + tagvar +'.eps',
                    format='eps', dpi=1000)

