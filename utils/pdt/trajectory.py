# =======================================================================================   
# More DMPs, a set of classes to play and learn with DMPs.
# dgerod@xyz-lab.org.es - 2015/16
# =======================================================================================

import numpy as np
from collections import namedtuple

# ---------------------------------------------------------------------------------------

class Trajectory(object):
    def __init__(self, numDim=0):
        self.Positions = []
        self._NumDim = numDim
    def clean(self):
        self.Positions = []

# ---------------------------------------------------------------------------------------

class CartTrajectory(Trajectory):

    def __init__(self):
        super(CartTrajectory,self).__init__(6)

    def poses(self):
        return np.array(self.Positions).squeeze()
    def setPoseCollection(self, poses):
        """ The collection must be [num_poses x 6]."""
        if len(poses[0]) < self._NumDim:
            raise ValueError("Incorrect dimensions")
        self.clean()
        for p in poses:
            self.Positions.append(p)
    def addPose(self, pose):
        if len(pose) < self._NumDim:
            raise ValueError("Incorrect dimensions")
        self.Positions.append(pose)

# ---------------------------------------------------------------------------------------

class PtpTrajectory(Trajectory):
    def __init__(self, numDim=7):
        super(PtpTrajectory,self).__init__(numDim)

    def joints(self):
        return np.array(self.Positions).squeeze()
    def setJointsCollection(self, joints):
        """ The collection must be [num_poses x num_joints]."""
        if len(joints[0]) < self._NumDim:
            raise ValueError("Incorrect dimensions")
        self.clean()
        for j in joints:
            self.Positions.append(j)
    def addJoints(self, joints):
        if len(joints) < self._NumDim:
            raise ValueError("Incorrect dimensions")
        self.Positions.append(joints)

# =======================================================================================
