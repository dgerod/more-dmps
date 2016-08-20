# -*- coding: utf-8 -*-

import numpy as np
import math

class CanonicalSystem:
    def __init__(self, decay=-2):
        # rate that the canonical part decays
        self.decay = decay
        
    def response(self, time):
        return math.exp(self.decay*time)
        
class Gaussian:
    def __init__(self, center=2, variance=1):
        self.center = center
        self.variance = variance
        
    def response(self, x):
        return math.exp(-self.variance*(x-self.center)**2)
                
class ForcingFunction:
    """
    Nonlinear forcing function that drives the system to imitate a behavior
    @params
        nbfs = number of basis functions
    """
    def __init__(self, nbfs=100):

        # rate that the forcing function's canonical part decays
        self.cs = CanonicalSystem(-2)

        self.nbfs = nbfs
        self.basisFunctions = []
        for x in range(0,nbfs):
            self.basisFunctions.append(Gaussian())

        self.weights = [1] * self.nbfs
        self.fd = []
        
    def _generateCentersAndVariance(self):
        pass
    
    def _computeWeights(self):
        pass
            
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
        
        # Generate function to interpolate the desired trajectory
        #import scipy.interpolate
        #path = np.zeros((self.dmps, self.timesteps))
        #x = np.linspace(0, self.cs.run_time, y_des.shape[1])
        #for d in range(self.dmps):
        #    path_gen = scipy.interpolate.interp1d(x, y_des[d])
        #    for t in range(self.timesteps):  
        #        path[d, t] = path_gen(t * self.dt)
        #y_des = path
        
        # CENTERS AND VARIANCES
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
        self.fd = Fd
       
        # COMPUTE WEIGHTS
        # self.computeWeights()
       
        # Find WEIGHTS for each BASIS FUNCTION by finding PSIs and Ss        
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
            self.weights[bfIdx] = np.transpose(S)*P*np.transpose(Fd)/(np.transpose(S)*P*S)
            bfIdx += 1
                        
    def response(self, time):
        """
        Return forcing function value for an specific time, it is obtained using 
        the weights computed from a previous example.
        """
        responseWithoutWeight = 0
        responseWithWeight = 0
        i = 0
        for basisFunction in self.basisFunctions: 
            responseWithoutWeight += basisFunction.response(self.cs.response(time))
            responseWithWeight += self.weights[i]*basisFunction.response(self.cs.response(time))
            i += 1
            
        # TODO ADD SCALING
        return (responseWithWeight/responseWithoutWeight)*self.cs.response(time)
    
    def responseToTimeArray(self, timeArray):
        totalResponse = np.array([])
        for time in timeArray:
            totalResponse = np.append(totalResponse, self.response(time))
        return totalResponse
