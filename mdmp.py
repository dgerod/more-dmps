# -*- coding: utf-8 -*-
'''
Implementation of Dynamical Motor Primitives (DMPs) for multi-dimensional
trajectories.
'''

import numpy as np
from dmp import DMP

class mDMP(object):
  '''
  Implementation of a Multi DMP (mDMP) as composition of several 
  Single DMPs (sDMP). This type of DMP is used with multi-dimensional 
  trajectories.
  ''' 
  def __init__(self, dim, nbfs):
        '''
        dim int: number of coordinates of the trajectory
        nbfs int: number of basis functions per sDMP
        '''
        assert (dim >= 1), "At least one dimension!"
        
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
        for sdx in xrange(self.dim):
            sdmp = self.dmps[sdx]
            sdmp.learn(trajectory[sdx,:], time)

        self.ns = self.dmps[0].ff.ns 
        self.W = self._weights()
                     
  def planNewTrajectory(self, start, goal, time):
      
        ns = int(time/self.dmps[0].stepTime) 
                
        pos = np.zeros((self.dim, ns))  
        vel = np.zeros((self.dim, ns))   
        acc = np.zeros((self.dim, ns))
        
        for sdx in xrange(self.dim):
            sdmp = self.dmps[sdx]
            
            sdmp.setup(start[sdx], goal[sdx])
            sdmp.plan(time)

            pos[sdx,:] = np.array(np.squeeze(sdmp.responsePos[1:]))                      
            vel[sdx,:] = np.array(np.squeeze(sdmp.responseVel[1:]))                      
            acc[sdx,:] = np.array(np.squeeze(sdmp.responseAccel[1:]))                      
  
        self.Fd, self.Fp = self._fs()
        return pos, vel, acc

# -----------------------------------------------------------------------------

from simple_cs import CanonicalSystem
from transformation_system import TransformationSystem
        
class sDMP(object):
    '''
    A Single Dynamical Movement Primitive (sDMP), it is used with one dimension
    trajectories. And it is composed by an attractor and a forcing funtion.
    '''
    def __init__(self, Dmp):
        self.cs = CanonicalSystem(-2)
        self.ts = TransformationSystem()
    
    def learn(self, trajectory, time):
        self.exampleTraj = trajectory
        self.exampleTime = time
        self.attractor.setEqPoint(self.exampleTraj[-1])
        
        self.ts.init(self.cs, self.attractor)
        self.ts.train(self.exampleTraj, self.exampleTime)
  
         
    def plan(self, time):
        pass
