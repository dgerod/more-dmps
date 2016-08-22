# -*- coding: utf-8 -*-

from abc import ABCMeta, abstractmethod

class TransfSystem(object):
    '''
    Transformation system of a sDMP, it is used by a sDMP for (1) learn information
    from a trajectory example and (2) to predict a new trajectory used the learned 
    model.
    '''
    __metaclass__ = ABCMeta

    def configure(self, cs):
        '''
        cs CanonicalSystem: the canonical system used by the transformation system.
        '''
        self.cs = cs        
    @abstractmethod
    def train(self, trajectory, time):
        '''
        Learn the model from a example trajectory.
        '''
        pass    
    @abstractmethod
    def predict(self, start, goal, time):
        '''
        Generate a new new trajectory using the learned model.
        '''    
        pass
