# -*- coding: utf-8 -*-

import math
import numpy as np
        
class Gaussian(object):
    def __init__(self, center=2, variance=1):
        self.center = center
        self.variance = variance
        
    def response(self, x):
        return math.exp(-self.variance*(x-self.center)**2)
                
class ShapeFunction(object):
    """
    Nonlinear forcing function that drives the system to imitate a behavior
    @params
        @params
        nbfs = number of basis functions
        cs = CanonicalSystem used by the function
    """
    def __init__(self, cs=None, nbfs=100):

        self.cs = cs

        self.nbfs = nbfs
        self.basisFunctions = []
        for x in range(0,nbfs):
            self.basisFunctions.append(Gaussian())

        self.weights = [1] * self.nbfs        
        self.Fd = None
        self.ns = 0

    def _generateCentersAndVariance(self):
        pass
    
    def _computeWeights(self, time):
        pass
    
    def _checkOffsetAndCorrect(self):
        """Check to see if initial position and goal are the same
        if they are, offset slightly so that the forcing term is not 0"""

        if (self.y0 == self.goal):
            self.goal += 1e-4
            
    def train(self, attractor, example, time):
        """
        Provide the forcing function an example and an attractor to modify the 
        combined forcing function and attractor to match the example.
        
        ASSUME THAT THE EXAMPLE IS INDEXED BY TIME AND STARTS AT 0
        """
        
        # Set initial state and goal
        y0 = example[0]
        g = example[-1]
        num_samples = len(example)
        #self.ns = num_samples  
        
        # Resample example to amount of desired samples
        #import scipy.interpolate
        #path = np.zeros((self.timesteps))
        #x = np.linspace(0, self.cs.run_time, num_samples)
        #path_gen = scipy.interpolate.interp1d(x, example)
        #for t in range(self.timesteps):  
        #   path[t] = path_gen(t * self.dt)
        #new_example = path

        #example = new_example
        #self.ns = len(new_example)
                 
        # CENTERS AND VARIANCES
        # -----------------------------------------
        # self._generateCentersAndVariance()
        
        # SET CENTERS
        # spread the basis functions evenly (set centers of gaussians)
        # done using studywolf blog's example 
        finalTime = time[-1]
        increment = finalTime/(self.nbfs+2) # dont place basis function at start or end
        
        i = 1
        for basisFunction in self.basisFunctions:
            basisFunction.center = self.cs.response(i*increment)
            i += 1
            
        # SET VARIANCES
        for basisFunction in self.basisFunctions:
            basisFunction.variance = self.nbfs/basisFunction.center
    
        # FIND Fd
        # -----------------------------------------
        # self._calculateDesiredForcingFunction()

        # First, calculate velocity and acceleration assuming uniform sampling 
        # for the example.

        exampleVel = np.diff(example)/(time[1]-time[0])
        exampleAccel = np.diff(exampleVel)/(time[1]-time[0])
        exampleVel = np.append(exampleVel, exampleVel[-1])
        exampleAccel = np.append(exampleAccel, [exampleAccel[-1], exampleAccel[-1]])
        
        # Find the force required (desired) to move along this trajectory        

        i = 0
        attractorAccel = np.array([])        
        for i in range(num_samples):
            response = attractor.response(example[i], exampleVel[i])
            attractorAccel = np.append(attractorAccel,response) 

        Fd = exampleAccel - attractorAccel
        
        self.Fd = Fd
        self.ns = len(Fd)
        
        # TODO-ADD: Here solve Fd using original and improved equations.
        # Replace previous calculus because, it is too theoretical.
       
        # COMPUTE WEIGHTS or MODEL (e.g. LWR)
        # -----------------------------------------
        # self.computeWeights() or self.computeModel()

        # Find WEIGHTS for each BASIS FUNCTION by finding PSIs and Ss
        weights = [1] * self.nbfs
        bfIdx = 0 # basis function index
        for basisFunction in self.basisFunctions:
            P = np.zeros((num_samples, num_samples))
            S = np.zeros((num_samples, 1))
            
            i = 0
            for datum in example:
                S[i,:] = self.cs.response(time[i])*(g-y0) # front_term                             
                P[i,i] = basisFunction.response(self.cs.response(time[i])) # psi
                i += 1
                
            S = np.mat(S)
            P = np.mat(P)
            Fd = np.mat(Fd)
            weights[bfIdx] = np.transpose(S)*P*np.transpose(Fd)/(np.transpose(S)*P*S)
            bfIdx += 1
        
        self.weights = np.array(np.squeeze(weights))
        
    def response(self, time):
        """
        Return forcing function value for an specific time, it is obtained using 
        the weights computed from a previous example.
        """
        
        # TODO-ADD: This is prediction, so here Fp should be calculated using original 
        # and improved equations.
        
        responseWithoutWeight = 0
        responseWithWeight = 0
        i = 0
        for basisFunction in self.basisFunctions: 
            responseWithoutWeight += basisFunction.response(self.cs.response(time))
            responseWithWeight += self.weights[i]*basisFunction.response(self.cs.response(time))
            i += 1
            
        # TODO-ADD: SCALING
        return (responseWithWeight/responseWithoutWeight)*self.cs.response(time)
    
    def responseToTimeArray(self, timeArray):
        totalResponse = np.array([])
        for time in timeArray:
            totalResponse = np.append(totalResponse, self.response(time))
        return totalResponse
