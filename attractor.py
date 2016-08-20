# -*- coding: utf-8 -*-

import math

class Attractor:    
    '''
    Represents a simple spring-damper system: 
        K(g-x) - Dv
    '''
    def __init__(self, proportional=10, eqPoint=1):
        '''
        proportional float: K
        eqPoint float: Goal (g)
        '''
        
        self.proportional = proportional # K parameter
        self.damping = math.sqrt(self.proportional) * 2 # D parameter
        
        # initialize states to 0
        self.xp = 0
        self.xv = 0
        self.xa = 0

        self.root = -math.sqrt(self.proportional)
        self.eqPoint = eqPoint

    def perturb(self, position=0, velocity=0):
        self.xp = position
        self.xv = velocity

    def timeResponse(self,t):
        c1 = self.xp
        c2 = self.xv
        return math.exp(self.root*t)*(c1 + c2*t)        

    def response(self, x, v, dt=.1):        
        """
        Function to obtain acceleration response        
        
        args
            x
            v
            dt = increment of time to update system
        """
        return -self.proportional*(x-self.eqPoint) - self.damping*v
        
    def setEqPoint(self, eqPoint):
        self.eqPoint = eqPoint
