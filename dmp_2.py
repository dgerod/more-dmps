
import numpy as np
      
from transformation_system import TransfSystem
from simple_cs import CanonicalSystem
from attractor import GoalAttractor
from shape_function import ShapeFunction

class TransformationSystem(TransfSystem):
    def __init__(self, cs=CanonicalSystem(-2), nbfs=100):
        '''
        cs CanonicalSystem: canonical system used by the shape function.
        nbfs int: number of basis functions
        '''

        self.cs = cs
        self.attractor = GoalAttractor()
        self.sf = ShapeFunction(self.cs, nbfs)
        
        self.exampleTraj = np.array([])
        self.exampleTime = np.array([])
        self.stepTime = .01 # step time learn
        
        self.start = 0.0
        self.goal = 0.0
        self.tau = 1.0
 
        self.stepNumber = 0
        self.responseTime = np.array([0])
        self.responseAccel = np.array([0])
        self.responseVel = np.array([0])
        self.responsePos = np.array([0])

    def _checkOffset(self):
        """Check to see if initial position and goal are the same
        if they are, offset slightly so that the forcing term is not 0"""

        # TODO-ADD: This check must be added in this class in the shape
        # function, too.       
       
        if (self.y0 == self.goal):
            self.goal += 1e-4
                
    def _setExample(self, trajectory, time):
        self.exampleTraj = trajectory
        self.exampleTime = time        
        self.start = trajectory[0]
        self.goal = trajectory[-1]
        
    def _imitate(self):
        self.sf.train(self.attractor, self.exampleTraj, self.exampleTime)
    
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
        newAccel = self.attractor.response(currentPos, currentVel, self.stepTime) + self.sf.response(self.stepNumber * self.stepTime)        
        self.responseAccel = np.append(self.responseAccel, newAccel)        
        
        newTime = self.stepNumber*self.stepTime
        self.responseTime = np.append(self.responseTime, newTime)
        
        self.stepNumber += 1
        
    def reset(self):
        pass
        
    def train(self, trajectory, time):
        self._setExample(trajectory, time)
        self._imitate()

    def setup(self, start, goal, duration=1.0):
        self.start = start
        self.goal = goal
        self.tau = duration
                
    def predict(self, start, goal, time):
        self.stepNumber = 0
        totalSteps = int(time/self.stepTime)        
        while self.stepNumber < totalSteps:
            self._step()
        return self.responsePos, self.responseVel, self.responseAccel
