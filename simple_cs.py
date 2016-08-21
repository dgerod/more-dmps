import math

class CanonicalSystem(object):
    def __init__(self, decay=-2):
        # rate that the canonical part decays
        self.decay = decay
        
    def response(self, time):
        return math.exp(self.decay*time)
        