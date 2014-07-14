# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa

"""
dp = True
import numpy as np
#from scipy.integrate import odeint
import logging,sys
logging.basicConfig(stream=sys.stderr,level=logging.DEBUG)

class H:
    def __init__(self, Q, states):
        self.q = Q #list of q
        self.states = states

    def sim(self,qID,X,t0,tlim,Ts=1e-4):
        #to refers to the initial time of
        #each continuous dynamic time interval
        sr = SimResult() #Initialize class
        q = self.q[qID] #get a ref to current mode

        while t0<tlim:

            #get values from current q object
            f=q.f   #continuous dynamics func
            g=q.E.G #guard list
            r=q.E.R #reset map list
            oe=q.E.OE #out edges list
            dom=q.Dom #discrete mode domain

            print 'check domain'

            errorString = 'Outside domain! \n X = %d \n qID = %d' % (X,qID)
            if not dom(X):
                raise NameError(errorString)
                print 'error!'

            #simulate continuous dynamics
            T,Y,oID_activated_g=odeeul(f,g,X,t0,Ts)
            # *** after guard is activated ***
            # prepare data for the next loop
            t0=T[-1] #reset initial time to the end of
                     #last time interval
            last_state = np.array(Y[-1])
            print 'out guard: ',oID_activated_g
            X=r[oID_activated_g](last_state) #reset map
            qID_activated_g = oe[oID_activated_g]
            print 'out edge: ',qID_activated_g
            q=self.q[qID_activated_g] #get new q

            logging.debug(X)

            #store this time interval
            #in the simulation results
            sr.newTimeInterval(T,Y)

        return sr

class Q:
    def  __init__(self,qID,f,E,
                  Init=True, Dom = lambda X:True,TC=True):
        self.qID = qID
        self.f = f
        self.E = E
        self.Init = Init
        self.Dom = Dom
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


def odeeul(f,g,X0,t0,Ts):
    X=np.array(X0)
    Y=np.array(X0)
    T=np.array([t0])

    g_activated,oID_activated_g = guard_check(g,X)
    while not g_activated:
        #Evolve continuosly until a guard is activated
        X=Ts*f(X)+X
        Y=np.vstack([Y,X])
        tnew = np.array([T[-1]+Ts])
        T=np.concatenate([T,tnew])
        g_activated,oID_activated_g = guard_check(g,X)

    return [T,Y,oID_activated_g]

class SimResult:
    def __init__(self):
        self.I= list()
        self.j=0
        self.timesteps=0

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











