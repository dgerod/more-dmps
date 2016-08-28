# -*- coding: utf-8 -*-
'''
Implementation of Dynamical Motor Primitives (DMPs) for multi-dimensional
trajectories.
'''

import numpy as np
from dmp_1 import DMP

class mDMP(object):
  '''
  Implementation of a Multi DMP (mDMP) as composition of several 
  Single DMPs (sDMP). This type of DMP is used with multi-dimensional 
  trajectories.
  ''' 
  def __init__(self, dim=1, nbfs=100):
        '''
        dim int: number of coordinates of the trajectory >= 1.
        nbfs int: number of basis functions per sDMP >= 0.
        '''
        
        self.dmps = [DMP(nbfs) for _ in xrange(dim)]
        self.dim = dim 
        self.nbfs = nbfs       
        self.ns = 0
                
  def _weights(self):
      
        W = np.zeros((self.dim, self.nbfs))        
        for sdx in xrange(self.dim):
            sdmp = self.dmps[sdx]
            W[sdx,:] = np.array(np.squeeze(sdmp.ff.weights))                      
        return W.T      
  
  def _fs(self):
      
        Fd = np.zeros((self.dim, self.ns))        
        Fp = np.zeros((self.dim, self.ns))        
        for sdx in xrange(self.dim):
            sdmp = self.dmps[sdx]
            Fd[sdx,:] = np.array(np.squeeze(sdmp.ff.Fd))        
            # TODO: Next line is patch as response time has 1 extra sample.
            time = sdmp.responseTime
            Fp[sdx,:] = np.array(np.squeeze(sdmp.ff.responseToTimeArray(time)))                      
        return Fd.T, Fp.T 
        
  def learnFromDemo(self, trajectory, time):      
        '''
        trajectory np.array([]): trajectory example (NxM).
        time np.array([]): time of the trajectory (NxM).
        '''
        
        for sdx in xrange(self.dim):
            sdmp = self.dmps[sdx]
            sdmp.learn(trajectory[sdx,:], time)

        self.ns = self.dmps[0].ff.ns 
        self.W = self._weights()
                     
  def planNewTrajectory(self, start, goal, time):
        '''
        start float: start positio of the new trajectory.
        goal float: end positio of the new trajectory.
        time float: time to execute the new trajectory.
        '''  
        
        ns = int(time/self.dmps[0].stepTime) 
                
        pos = np.zeros((self.dim, ns))  
        vel = np.zeros((self.dim, ns))   
        acc = np.zeros((self.dim, ns))
        
        for sdx in xrange(self.dim):
            sdmp = self.dmps[sdx]
            
            sdmp.setup(start[sdx], goal[sdx])
            sdmp.plan(time)

            # TODO: Next line is patch as response time has 1 extra sample.
            pos[sdx,:] = np.array(np.squeeze(sdmp.responsePos[1:]))                      
            vel[sdx,:] = np.array(np.squeeze(sdmp.responseVel[1:]))                      
            acc[sdx,:] = np.array(np.squeeze(sdmp.responseAccel[1:]))                      
  
        self.Fd, self.Fp = self._fs()
        return pos, vel, acc

# -----------------------------------------------------------------------------

from simple_cs import CanonicalSystem
from transformation_system import TransfSystem

class sDMP(object):
    '''
    A Single Dynamical Movement Primitive (sDMP), it is used with one dimension
    trajectories. And it is composed by a transformation system and a canonical system.
    '''
    def __init__(self, tf):
        '''
        tf TransfSystem: the transformation system of a DMP.
        '''
        
        self.cs = CanonicalSystem(-2)
        self.ts = tf
        self.ts.configure(self.cs)
        
        self.exampleTraj = np.array([])
        self.exampleTime = np.array([])
        
        self.start = 0.0
        self.goal = 0.0
        self.tau = 1.0
    
    def learn(self, trajectory, time):
        '''
        trajectory np.array([]): trajectory example (1xM).
        time np.array([]): time of the trajectory (1xM).
        '''
        
        self.exampleTraj = trajectory
        self.exampleTime = time
        self.ts.train(self.exampleTraj, self.exampleTime)
        
    def plan(self, start, goal, time): 
        '''
        start float: start positio of the new trajectory.
        goal float: end positio of the new trajectory.
        time float: time to execute the new trajectory.
        '''
        
        self.start = start
        self.goal = goal        
        return self.ts.predict(self.start, self.goal, time)

# -----------------------------------------------------------------------------

class mDmpUsing1stFormulation:
    pass

class mDmpUsing2ndFormulation:
    pass

class mDmpUsing3rdFormulation:
    pass
