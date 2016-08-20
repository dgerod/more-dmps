# -*- coding: utf-8 -*-
'''
Implementation of Dynamical Motor Primitive (DMP).
'''

import numpy as np

from attractor import Attractor 
from forcing_function import ForcingFunction

class mDMP:
  '''Implementation of a Multi DMP (mDMP) as composition of several 
  Single DMPs (sDMP). This type of DMP is used with multi-dimensional trajectories.'''
  def __init__(self, dim=1, nbfs=100):
        '''
        dim int: number of coordinates of the trajectory
        nbfs int: number of basis functions per sDMP
        '''
        
        self.dim = dim 
        self.nbfs = nbfs        
        self.dmps = [DMP(self.nbfs) in xrange(self.dim)]

  def learnFromDemo(self, trajectory, time):
        for sdmp in self.dmps:
            sdmp.setExample(trajectory, time)
            sdmp.imitate()
                
  def planNewTrajectory(self, time):
        for sdmp in self.dmps:
            sdmp.run(time)
        
class DMP:
    '''A Single Dynamical Movement Primitive (sDMP), it is used with one dimension
    trajectories. And it is composed by an attractor and a forcing funtion'''
    def __init__(self, nbfs=100):
        '''
        nbfs int: number of basis functions
        '''
        
        self.ff = ForcingFunction()
        self.attractor = Attractor()
        
        self.example = np.array([])
        self.exampleTime = np.array([])
        
        self.responseAccel = np.array([0])
        self.responseVel = np.array([1])
        self.responsePos = np.array([0])
        
        self.stepTime = .01 #step timelearn
        self.stepNumber = 0
        
    def _discreteDefIntegral(self, sequence, startIndex, endIndex):
        return np.sum(sequence[startIndex:endIndex+1])
    
    def setExample(self, trajectory, time):
        self.example = trajectory
        self.exampleTime = time
        #self.attractor.eqPoint = trajectory[-1]
        self.attractor.setEqPoint(trajectory[-1])
        
    def imitate(self):
        self.ff.train(self.attractor, self.example, self.exampleTime)
    
    def step(self):         
        # UPDATE STATE
        currentPos = self.responsePos[-1]
        currentVel = self.responseVel[-1]
        currentAccel = self.responseAccel[-1]
        
        newVel = currentVel + currentAccel*self.stepTime
        self.responseVel = np.append(self.responseVel,newVel)
        
        newPos = currentPos + currentVel*self.stepTime
        self.responsePos = np.append(self.responsePos,newPos)
        
        # CALCULATE NEW ACCEL  
        newAccel = self.attractor.response(currentPos, currentVel, self.stepTime) + self.ff.response(self.stepNumber * self.stepTime)        
        self.responseAccel = np.append(self.responseAccel, newAccel)        
        
        self.stepNumber += 1
            
    def run(self, timeToRun):
        while (self.stepNumber*self.stepTime) < timeToRun:
            self.step()  
 
    def learn(self, trajectory, time):
        self.example = trajectory
        self.exampleTime = time
        self.attractor.setEqPoint(trajectory[-1])
        self.ff.train(self.attractor, self.example, self.exampleTime)
                
    def plan(self, time):
        self.run(time)
