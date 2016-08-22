# -*- coding: utf-8 -*-

from mdmp import sDMP
import numpy as np
import matplotlib.pyplot as plt

from dmp_2 import TransformationSystem as MyTransfSystem

t = np.arange(0, np.pi/2, 0.01)
y = np.sin(t)

sdmp = sDMP(MyTransfSystem(nbfs=100))
sdmp.learn(y, t)

fp = sdmp.ts.sf.responseToTimeArray(t)
plt.figure()
plt.title('Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(sdmp.ts.sf.Fd,'r--', label='Fd')
plt.plot(fp,'g-', label='Fp')
plt.show()
        
y2, y2d, y2dd = sdmp.plan(y[0], y[-1], np.pi/2)

plt.figure()
plt.title('Trajectory - Demo (Td) and generated (Tp)')
plt.plot(y, 'r--', label='Td')
plt.plot(y2,'g-', label='Tp')
plt.show()
