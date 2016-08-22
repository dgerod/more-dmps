# -*- coding: utf-8 -*-

from dmp_1 import DMP
import numpy as np
import matplotlib.pyplot as plt

t = np.arange(0, np.pi/2, 0.01)
y = np.sin(t)

plt.figure()
plt.title('Demo (X, Y)')
plt.plot(y,'g-')
plt.show()

# Learn dmp
sdmp = DMP(nbfs=100)
sdmp.learn(y, t)

plt.figure()
plt.title('Weights (X, Y)')
plt.plot(sdmp.ff.weights,'b-')
plt.show()

Fp = sdmp.ff.responseToTimeArray(t)
plt.figure()
plt.title('Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(sdmp.ff.Fd,'r--', label='Fd')
plt.plot(Fp,'b-', label='Fp')
plt.show()

# Plan a new trajectory 
sdmp.plan(np.pi/2)
result = sdmp.responsePos

plt.figure()
plt.title('Trajectory - Demo (Td) and generated (Tp)')
plt.plot(y, 'r--', label='Tp')
plt.plot(result,'g-', label='Td')
plt.show()
