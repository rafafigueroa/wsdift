#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""
dp = True
import numpy as np
#from scipy.integrate import odeint

DEBUG = False

class H:
    def __init__(self, Q, states):
        self.q = Q #list of q
        self.states = states

    def sim(self,qID,X,u,t0,tlim,debug_flag=False,Ts=1e-4):
        #to refers to the initial time of
        #each continuous dynamic time interval
        sr = SimResult() #Initialize class
        q = self.q[qID] #get a ref to current mode
        global DEBUG
        DEBUG = debug_flag #change global DEBUG variable

        while t0<tlim:

            #get values from current q object
            f=q.f   #continuous dynamics func
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
            T,Y,oID_activated_g,avoid_activated= \
            odeeul(f,g,avoid,X,u,t0,Ts)
            print 'guard oID: ',oID_activated_g
            print 'inside avoid set? ',avoid_activated
            
            #store this time interval
            #in the simulation results
            sr.newTimeInterval(T,Y)
            
            #when inside the avoid set, simulation stops
            #and the information is stored in the simulation results
            if avoid_activated:
                sr.avoid_activated = True
                sr.timeToAvoid = T[-1]
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
            #normal print out
            print 'from q =',q.qID,'to q =',qID_activated_g
            print 'State =',X
            q=self.q[qID_activated_g] #get new q

        return sr

class Q:
    def  __init__(self,qID,f,E,
                  Init=True, Dom = lambda X:True, Avoid = lambda X:False ,TC=True):
        self.qID = qID
        self.f = f
        self.E = E
        self.Init = Init
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

    return [g_activated,oID_activated_g]
    
def avoid_check(avoid,X):
    'avoid returns True when inside the avoid set'
    return avoid(X)
    


def odeeul(f,g,avoid,X0,u,t0,Ts):
    #TODO: change constant u for function option
    X=np.array(X0)
    Y=np.array(X0)
    T=np.array([t0])

    if DEBUG:
        print 'State=',X

    g_activated,oID_activated_g = guard_check(g,X)
    avoid_activated = avoid_check(avoid,X)
        
    if DEBUG:
        print 'First checks:'
        print '\tg_activated:',g_activated
        print '\tavoid_activated',avoid_activated
    
    while not (g_activated or avoid_activated):
        #Evolve continuosly until a guard is activated
        
        X=Ts*f(X,u)+X
        #print X
        
        Y=np.vstack([Y,X])
        tnew = np.array([T[-1]+Ts])
        T=np.concatenate([T,tnew])
        g_activated,oID_activated_g = guard_check(g,X)
        avoid_activated = avoid_check(avoid,X)
    
    print 'Output=',Y
    return [T,Y,oID_activated_g,avoid_activated]

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











