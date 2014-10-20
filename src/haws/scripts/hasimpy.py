#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
dp = True
import numpy as np

DEBUG = False

class H:

    def __init__(self, Q, Init_X, Init_qID, states):
        self.q = Q #list of q
        self.Init_X = Init_X
        self.Init_qID = Init_qID
        self.states = states

    def mode_tracker_guard_check(self, qID, X):
        # Called by mode_tracker to set the mode
        q = self.q[qID]
        g=q.E.G #guard list
        oe=q.E.OE #out edges list
        [g_activated, oID_activated_g] = guard_check(g, X)

        # return new qID when a guard is activated
        if g_activated:
            qID_activated_g = oe[oID_activated_g]
        else:
            qID_activated_g = qID

        return qID_activated_g

    def sim(self, qID, X, u, t0, tlim,
    haws_flag=False,
    debug_flag=False,Ts=1e-4):
        #t0 refers to the initial time of
        #each continuous dynamic time interval

        sr = SimResult() #Initialize class
        q = self.q[qID] #get a ref to current mode
        global DEBUG
        DEBUG = debug_flag #change global DEBUG variable

        while t0<tlim:

            #get values from current q object
            f=q.f   #continuous dynamics func
            # when simulating is requested by haws
            # with a forced input
            if not haws_flag:
                u=q.u

            g=q.E.G #guard list
            r=q.E.R #reset map list
            oe=q.E.OE #out edges list
            dom=q.Dom #discrete mode domain
            avoid=q.Avoid #discrete mode avoid
            
            if DEBUG:
                print '\n*** New Discrete State *** \n'
                print 'f=',f,'\ng=',g,'\nr=',r,'\noe=',oe,'\ndom=',dom
                print 'Avoid=',avoid 
                print 'qID=',q.qID,'\nX=',X,'\nu=',u
                print '\n*** domain check *** \n'

            if not dom(X):
                errorString = 'Outside domain!'
                print errorString
                #raise NameError(errorString)

            if DEBUG:
                print '\n*** continuous dynamics *** \n'
                
            #simulate continuous dynamics
            T, Y, oID_activated_g, \
            avoid_activated, tlim_activated = \
            odeeul(f, u, g, avoid, X, t0, tlim, Ts)

            # force single row outputs to maintain general
            # 2D matrix form (I miss matlab here)
            Y = np.array(Y, ndmin = 2)

            # store this time interval
            # in the simulation results
            sr.newTimeInterval(T,Y)
            
            # when inside the avoid set, simulation stops
            # and the information is stored in the simulation results
            if avoid_activated:
                sr.avoid_activated = True
                sr.timeToAvoid = T[-1]
                break #while loop
                
            if tlim_activated:
                break #while loop

            # *** after guard is activated ***

            # prepare data for the next loop
            t0=T[-1] #reset initial time to the end of
                     #last time interval

            last_state = np.array(Y[-1])

            if DEBUG:
                print '\n *** reset map *** \n'
                print 'last state =',last_state
                
            X=r[oID_activated_g](last_state) #reset map
            qID_activated_g = oe[oID_activated_g]

            #guard activated print out
            print 'sim -- guard activated'
            print 'sim -- from q =', q.qID, 'to q =', qID_activated_g
            print 'sim -- State =', X

            #get new q
            q = self.q[qID_activated_g]

        return sr


class Q:

    def  __init__(self,qID,f,u,E,
                  Dom = lambda X:True,
                  Avoid = lambda X:False ,
                  TC=True):
        self.qID = qID
        self.f = f
        self.u = u
        self.E = E
        self.Dom = Dom
        self.Avoid = Avoid
        self.TC = TC


class E:

    def __init__(self,OE,G,R):
        self.OE = OE
        self.G = G
        self.R = R

def guard_check(g,X):
    guard_list = []
    #evaluate every guard in g
    #g is the list of guards for this q
    #store the results in guard_list
    for guard in g:
        guard_list.append(guard(X))

    oID_activated_g = None
    g_activated = False

    #check if any result in guard_list is True
    #if it is, store the index
    for oID,guard in enumerate(guard_list):
        if guard:
            oID_activated_g = oID #outside q which tripped the guard
            g_activated = True
            break

    return [g_activated, oID_activated_g]

def avoid_check(avoid,X):
    'avoid returns True when inside the avoid set'
    return avoid(X)

def odeeul(f, u, g, avoid, X0, t0, tlim, Ts):
    X=np.array(X0)
    Y=np.array(X0)
    T=np.array([t0])

    if DEBUG:
        print 'State=',X

    g_activated, oID_activated_g = guard_check(g,X)
    avoid_activated = avoid_check(avoid,X)
    tlim_activated = (t0>=tlim)

    if g_activated:
        print 'instant jump'

    if DEBUG:
        print 'First checks:'
        print '\tg_activated:', g_activated
        print '\tavoid_activated', avoid_activated
        print '\ttlim_activated', tlim_activated
    
    while not (g_activated or avoid_activated or tlim_activated):
        #Evolve continuously until a
        #termination condition is activated
        
        X=Ts*f(X,u)+X

        Y=np.vstack([Y,X])
        tnew = np.array([T[-1]+Ts])
        T=np.concatenate([T,tnew])

        #termination checks
        g_activated, oID_activated_g = guard_check(g,X)
        avoid_activated = avoid_check(avoid,X)
        tlim_activated = (tnew>=tlim)
        
        if DEBUG:
            print 'Running checks:'
            print '\tg_activated:',g_activated
            print '\tavoid_activated',avoid_activated
            print '\ttlim_activated',tlim_activated
        
    return [T, Y, oID_activated_g, avoid_activated,
            tlim_activated]

class SimResult:
    def __init__(self):
        self.I= list()
        self.j=0
        self.timesteps=0
        self.timeToAvoid=None
        self.avoid_activated=False

    def newTimeInterval(self,T,Y):
        self.j+=1
        self.timesteps+=np.size(T)
        self.I.append(TimeInterval(T,Y,self.j))

    def path(self):
        Ylist = []
        for i in self.I:
            Ylist.append(i.Y)

        return np.vstack(Ylist)


    def simPlot(self):

        Ylist=[]
        Tlist=[]
        for i in self.I:
            Ylist.append(i.Y)
            Tlist.append(i.T)

        Y_plot = np.vstack(Ylist)
        T_plot = np.concatenate(Tlist)

        import matplotlib.pyplot as plt

        nstates = np.size(Y_plot,1)
        f, axarr = plt.subplots(nstates, sharex=True)

        if nstates>1:
            for yi in range(nstates):
                axarr[yi].plot(T_plot, Y_plot[:,yi])
        else:
            axarr.plot(T_plot,Y_plot)

        plt.ion()
        plt.show()
        
    def phasePlot(self,plotStates):
            
        #TODO:check size of Y,plotStates
        X1_list=[]
        X2_list=[]
        for i in self.I:
            X1_list.append(i.Y[:,plotStates[0]])
            X2_list.append(i.Y[:,plotStates[1]])
        if DEBUG:
            print 'X1_list=' ,X1_list
            print 'X2_list=' ,X2_list

        X1_plot = np.concatenate(X1_list)
        X2_plot = np.concatenate(X2_list)

        import matplotlib.pyplot as plt
        f, axarr = plt.subplots(1, sharex=True)
        
        axarr.plot(X1_plot,X2_plot)
        
        plt.ion()
        plt.show()
        
 
        
class TimeInterval:
    def __init__(self,T,Y,j):
        self.T=T
        self.Y=Y
        self.j=j

def idem(X):
    return X
    
def tolEqual(a,b,tol=1e-2):
    return abs(a-b)<tol

def last_row(Y):
    print 'shape', np.shape(Y)

    rows = np.shape(Y)[0]
    print 'rows',rows
    if rows>1:
        return Y[-1]
    else:
        return Y

