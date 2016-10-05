# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

from dmp_1 import DMP

#
# -------------------------------

t1 = np.arange(0, np.pi/2, 0.01)
y1 = np.sin(t1)

plt.figure()
plt.title('Demo')
plt.plot(y1,'g-')
plt.show()

# Learn dmp
sdmp = DMP(nbfs=100)
sdmp.learn(y1, t1)

plt.figure()
plt.title('Weights')
plt.plot(sdmp.ff.weights,'b-')
plt.show()

Fp = sdmp.ff.responseToTimeArray(t1)
plt.figure()
plt.title('Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(sdmp.ff.Fd,'r--', label='Fd')
plt.plot(Fp,'b-', label='Fp')
plt.show()

# Plan a new trajectory 
sdmp.plan(np.pi/2)
result = sdmp.responsePos
y2 = sdmp.responsePos
dy2 = sdmp.responseVel
ddy2 = sdmp.responseAccel

plt.figure()
plt.title('Trajectory - Demo (Td) and generated (Tp)')
plt.plot(y1, 'r--', label='Tp')
plt.plot(y2,'g-', label='Td')
plt.show()

#
# -------------------------------

from utils.stg.min_jerk_traj import trajectory

duration = 1.0
delta_t = 0.001

duration = np.pi/2
delta_t = 0.01

t1 = np.arange(0, duration, delta_t)
y1 = trajectory(0.5, 1.0, duration, delta_t)

plt.figure()
plt.title('Trajectory - Demo (Td)')
plt.plot(np.matrix(y1)[:,0],'r-')
plt.plot(np.matrix(y1)[:,1],'g-')
plt.plot(np.matrix(y1)[:,2],'b-')
plt.show()

# Learn dmp
sdmp = DMP(nbfs=100)

y = np.squeeze(np.array(np.matrix(y1)[:,0]))
sdmp.learn(y, t1)

plt.figure()
plt.title('Weights')
plt.plot(sdmp.ff.weights,'b-')
plt.show()

Fp = sdmp.ff.responseToTimeArray(t1)
plt.figure()
plt.title('Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(sdmp.ff.Fd,'r--', label='Fd')
plt.plot(Fp,'b-', label='Fp')
plt.show()

# Plan a new trajectory 
sdmp.plan(duration)
y2 = sdmp.responsePos
dy2 = sdmp.responseVel
ddy2 = sdmp.responseAccel

plt.figure()
plt.title('Trajectory - Demo (Td) and generated (Tp)')
plt.plot(np.matrix(y1)[:,0],'g-', label='Tp')
plt.plot(y2, 'b-', label='Tp')

plt.show()
