# =============================================================================
# More DMPs, a set of classes to play and learn with DMPs.
# dgerod@xyz-lab.org.es - 2015/16
# -----------------------------------------------------------------------------
# This file is notebook that runs in Spyder/IPython.
#
# Notebook used to learn a Multi-DMP from a Cartesian trajectory, the trajectory
# is stored as a set of pose messages in a ROS bag.
# =============================================================================

# %% Load a trajectory from a bag
# -----------------------------------------------------------------------------

FileName = "C-Shape-1_poses.bag"

import os
from utils.rbag.poses import PosesBagLoader
from utils.pdt.trajectory import CartTrajectory

db_path = os.path.realpath(os.getcwd()) + "/trajectory_db/"
loader = PosesBagLoader()
poses = loader.read(db_path + FileName).poses()[:,0:2]

# %% Learn a dmp using original code from StudyWolf web.
# -----------------------------------------------------------------------------

import numpy as np
import matplotlib.pylab as plt
from third_party.tdw.dmp_discrete import DMPs_discrete

nbfs = 100
mdmp = DMPs_discrete(dims=poses.shape[1], bfs=nbfs)    
mdmp.imitate_path(y_des=poses.T)

plt.figure()
plt.plot(poses)
plt.figure()
plt.plot(mdmp.w.T)
plt.figure()
plt.plot(mdmp.f_target)
plt.show()

# %% Learn a dmp using equations from StudyWolf web.
# -----------------------------------------------------------------------------

import numpy as np
import matplotlib.pylab as plt
from dmp_4 import DMPs_discrete, TdwFormulation

nbfs = 100
mdmp = DMPs_discrete(dims=poses.shape[1], bfs=nbfs, ts=TdwFormulation(), dt=.02)    
mdmp.learn(y_des=poses.T)
        
plt.figure()
plt.plot(poses)
plt.figure()
plt.plot(mdmp.w.T)
plt.figure()
plt.plot(mdmp.f_desired)
plt.show()

# %% Learn a dmp using equations in paper XXX
# -----------------------------------------------------------------------------

import numpy as np
import matplotlib.pylab as plt
from dmp_4 import DMPs_discrete, OriginalFormulation

nbfs = 100
mdmp = DMPs_discrete(dims=poses.shape[1], bfs=nbfs, ts=OriginalFormulation())    
mdmp.learn(y_des=poses.T)

plt.figure()
plt.plot(poses)
plt.figure()
plt.plot(mdmp.w.T)
plt.figure()
plt.plot(mdmp.f_desired)
plt.show()

# %% Learn a dmp using equations in paper YYY
# -----------------------------------------------------------------------------

import numpy as np
import matplotlib.pylab as plt
from dmp_4 import DMPs_discrete, ImprovedFormulation

nbfs = 100
mdmp = DMPs_discrete(dims=poses.shape[1], bfs=nbfs, ts=ImprovedFormulation())    
mdmp.learn(y_des=poses.T)

plt.figure()
plt.plot(poses)
plt.figure()
plt.plot(mdmp.w.T)
plt.figure()
plt.plot(mdmp.f_desired)
plt.show()

# =============================================================================