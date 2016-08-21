# -*- coding: utf-8 -*-

from mdmp import mDMP
import numpy as np
import matplotlib.pyplot as plt

t1 = np.arange(0, np.pi/2, 0.01)
y1 = np.array([np.sin(t1),np.cos(t1)])

# Plot demo
plt.figure()
plt.title('Demo (X, Y)')
plt.plot(y1[0,:],'r-')
plt.plot(y1[1,:],'g-')
plt.show()

mdmp = mDMP(y1.shape[0], 100)
mdmp.learnFromDemo(y1, t1)

# Plot weights
plt.figure()
plt.title('Weights (X, Y)')
plt.plot(mdmp.W[:,0],'r-')
plt.plot(mdmp.W[:,1],'g-')
plt.show()

s = y1[:,0] 
g = y1[:,-1]
t = t1[-1]
y2, yd2, ydd2 = mdmp.planNewTrajectory(s, g, t)

# Plot forcing functions
plt.figure()
plt.title(' X - Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(mdmp.Fd[:,0],'r--', label='Fd')
plt.plot(mdmp.Fp[:,0],'g-', label='Fp')
plt.show()

plt.figure()
plt.title(' Y - Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(mdmp.Fd[:,1],'r--', label='Fd')
plt.plot(mdmp.Fp[:,1],'g-', label='Fp')
plt.show()

# Plot new trajectory
plt.figure()
plt.title(' Planned trajectory (X, Y)')
plt.plot(y2[0,:],'r-')
plt.plot(y2[1,:],'g-')
plt.show()

plt.figure()
plt.title('Trajectory (X) - Demo (Td) and generated (Tp)')
plt.plot(y1[0,:], 'r--', label='Td')
plt.plot(y2[0,:],'g-', label='Tp')
plt.show()
plt.figure()
plt.title('Trajectory (Y) - Demo (Td) and generated (Tp)')
plt.plot(y1[1,:], 'r--', label='Td')
plt.plot(y2[1,:],'g-', label='Tp')
plt.show()
