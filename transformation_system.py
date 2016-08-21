# -*- coding: utf-8 -*-

class TransformationSystem(object):    
    def __init__(self):
        pass

    def learn(self):
        pass
    
    def predict(self):
        pass
    
class OriginalFormulation(object):
    ''' original formulation '''

    @staticmethod
    def transformation(k_gain, d_gain, x, raw_xd, start, goal, tau, f, s):
        return (k_gain * (goal - x) - d_gain * raw_xd + (goal - start) * f) / tau
      
    @staticmethod
    def ftarget(k_gain, d_gain, y, yd, ydd, goal, start, tau, s):
        return ((-1 * k_gain * (goal - y) + d_gain * yd + tau * ydd) / (goal - start))

class ImprovedFormulation(object):
    ''' Improved version of formulation'''
    
    @staticmethod
    def transformation(k_gain, d_gain, x, raw_xd, start, goal, tau, f, s):
        return (k_gain * (goal - x) - d_gain * raw_xd - k_gain * (goal - start) * s + k_gain * f) / tau
      
    @staticmethod
    def ftarget(k_gain, d_gain, y, yd, ydd, goal, start, tau, s):
        # published formula
        #return ((tau * ydd - d_gain * yd) / k_gain ) + (goal - y) - ((goal - start) * s)
        # version from dmp_lib/transformation_systm.cpp
        return ((tau**2 * ydd + d_gain * yd * tau) / k_gain ) - (goal - y) + ((goal - start) * s)
