import numpy as np
import matplotlib.pyplot as plt
from utils.stg.min_jerk_traj import trajectory
  
start = 0.5
goal = 1.0
duration = 1.0
dt = 0.001

time = np.arange(0, duration, dt)
example = trajectory(start, goal, duration, dt)

plt.figure()
plt.title('Trajectory X - Demo (Td)')
plt.plot(np.matrix(example)[:,0],'r-')
plt.plot(np.matrix(example)[:,1],'g-')
plt.plot(np.matrix(example)[:,2],'b-')
plt.show()
 
