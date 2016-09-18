# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pylab as plt

from dmp_3 import *
from utils.stg.min_jerk_traj import trajectory
  
# start and goal of the movement (1-dim)
# trajectory is a list of 3-tuples with (pos,vel,acc)
  
start = 0.5
goal = 1.0
duration = 1.0
delta_t = 0.001

y = trajectory(start, goal, 1.0, delta_t)

plt.figure()
plt.title('Trajectory X - Demo (Td)')
plt.plot(np.matrix(y)[:,0],'r-')
plt.plot(np.matrix(y)[:,1],'g-')
plt.plot(np.matrix(y)[:,2],'b-')
plt.show()

# Learn dmp
dmp1 = DiscreteDMP(OriginalFormulation(), 
LWR(activation=0.3, exponentially_spaced=False, n_rfs=8, use_offset=True))
  
dmp2 = DiscreteDMP(ImprovedFormulation(), 
LWR(activation=0.3, exponentially_spaced=False, n_rfs=8, use_offset=True))
 
# Time is scalar but in the other implementations is a  array of time_steps.
time = 1.0
dmp1.learn(y, time)
dmp2.learn(y, time)
  
plt.figure()
plt.title('Original From - Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(dmp1.Fd,'r--', label='Fd')
plt.plot(dmp1.Fp,'b-', label='Fp')
plt.show()

plt.figure()
plt.title('Improvement From - Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(dmp2.Fd,'r--', label='Fd')
plt.plot(dmp2.Fp,'b-', label='Fp')
plt.show()

# Plan a new trajectory 
dmp1.use_ft = False
dmp1.setup(start+0.4, goal+0.2, 1.0)
y1, y1d, y1dd = dmp1.plan(time)

plt.figure()
plt.title('Original From - Trajectory X - Generated (Tp)')
plt.plot(y1,'r-')
plt.plot(y1d,'g-')
plt.plot(y1dd,'b-')
plt.show()
  
dmp2.use_ft = False
dmp2.setup(start+0.4, goal+0.2, 1.0)
y2, y2d, y2dd = dmp2.plan(time)
 
plt.figure()
plt.title('Improved From - Trajectory X - Generated (Tp)')
plt.plot(y2,'r-')
plt.plot(y2d,'g-')
plt.plot(y2dd,'b-')
plt.show()

plt.figure()
plt.title('Trajectory X - Error in position')
err1 = abs(np.squeeze(np.array(np.matrix(y)[:,0])).shape - y1)
err2 = abs(np.squeeze(np.array(np.matrix(y)[:,0])).shape - y2)
plt.plot(err1)
plt.plot(err2)
 
