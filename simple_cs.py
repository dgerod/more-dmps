
import math

class CanonicalSystem(object):
    '''
    A simple canonical system.
    '''
    def __init__(self, decay=-2.):
        '''
        decay float: rate that the canonical part decays.
        '''
        self.decay = decay
        
    def response(self, time):
        '''
        time float:
        '''                
        return math.exp(self.decay*time)
        