# -*- coding: utf-8 -*-

from dmp import DMP
import numpy as np
import matplotlib.pyplot as plt

t = np.arange(0, np.pi/2, 0.01)
y = np.sin(t)

sdmp = DMP()
sdmp.learn(y, t)

Fp = sdmp.ff.responseToTimeArray(t)
plt.figure()
plt.title('Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(sdmp.ff.Fd,'r--', label='Fd')
plt.plot(Fp,'g-', label='Fp')
plt.show()
        
sdmp.plan(np.pi/2)
result = sdmp.responsePos

plt.figure()
plt.title('Trajectory - Demo (Td) and generated (Tp)')
plt.plot(y, 'r--', label='Tp')
plt.plot(result,'g-', label='Td')
plt.show()
