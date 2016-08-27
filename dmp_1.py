# -*- coding: utf-8 -*-
'''
Implementation of Dynamical Motor Primitives (DMPs).
'''

import numpy as np

from simple_cs import CanonicalSystem
from attractor import GoalAttractor 
from shape_function import ShapeFunction
 
class DMP(object):
    '''
    A Single Dynamical Movement Primitive (sDMP), it is used with one dimension
    trajectories. And it is composed by an attractor and a forcing funtion
    '''    
    def __init__(self, nbfs=100):
        '''
        nbfs int: number of basis functions
        '''

        # Rate that the forcing function's canonical part decays       
        self.cs = CanonicalSystem(-2)
        
        # Transformation system
        self.attractor = GoalAttractor()
        self.ff = ShapeFunction(self.cs, nbfs)
        
        self.exampleTraj = np.array([])
        self.exampleTime = np.array([])
        self.stepTime = .01
        
        self.start = 0.0
        self.goal = 0.0
        self.tau = 1.0
 
        self.stepNumber = 0
        self.responseTime = np.array([0])
        self.responseAccel = np.array([0])
        self.responseVel = np.array([1])
        self.responsePos = np.array([0])
                
    def _discreteDefIntegral(self, sequence, startIndex, endIndex):
        return np.sum(sequence[startIndex:endIndex+1])
    
    def _setExample(self, trajectory, time):
        self.exampleTraj = trajectory
        self.exampleTime = time 
        
        self.start = trajectory[0]
        self.goal = trajectory[-1]
        self.attractor.setEqPoint(self.goal)
        
    def _imitate(self):
        self.ff.train(self.attractor, self.exampleTraj, self.exampleTime)
    
    def _step(self):   
        
        # UPDATE STATE
        currentPos = self.responsePos[-1]
        currentVel = self.responseVel[-1]
        currentAccel = self.responseAccel[-1]
        
        newVel = currentVel + currentAccel*self.stepTime
        self.responseVel = np.append(self.responseVel, newVel)
        
        newPos = currentPos + currentVel*self.stepTime
        self.responsePos = np.append(self.responsePos, newPos)
                
        # CALCULATE NEW ACCEL  
        # self.attractor.response(prevPos, prevVel, stepTime)
        # newAccel = self.ts.response(self.attractor, self.ff, stepTime)
        newAccel = self.attractor.response(currentPos, currentVel, self.stepTime) + \
            self.ff.response(self.stepNumber * self.stepTime)        
        self.responseAccel = np.append(self.responseAccel, newAccel)        
        
        newTime = self.stepNumber*self.stepTime
        self.responseTime = np.append(self.responseTime, newTime)
        
        self.stepNumber += 1
        
    def reset(self):
        pass
        
    def learn(self, trajectory, time):
        self._setExample(trajectory, time)
        self._imitate()

    def setup(self, start, goal, duration=1.0):
        self.start = start
        self.goal = goal
        self.tau = duration        
                  
    def plan(self, time):
        self.stepNumber = 0
        totalSteps = int(time/self.stepTime)        
        while self.stepNumber < totalSteps:
            self._step()   
