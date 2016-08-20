# -*- coding: utf-8 -*-

from dmp import DMP
import numpy as np
import matplotlib.pyplot as plt

t = np.arange(0, np.pi/2, 0.01)
y = np.sin(t)

sdmp = DMP()
sdmp.learn(y, t)

ft = sdmp.ff.responseToTimeArray(t)
plt.figure()
plt.title('Forcing Function: Desired (Fd) and trained (Ft)')
plt.plot(ft,'g-', label='Ft')
plt.plot(sdmp.ff.fd,'r--', label='Fd')
plt.show()
        
sdmp.plan(np.pi/2)
result = sdmp.responsePos

plt.figure()
plt.title('Trajectory - Demo (Td) and imitated (Ti)')
plt.plot(result)
plt.plot(y, 'g--')
plt.show()
