import numpy as np
import matplotlib.pyplot as plt
from simple_cs import CanonicalSystem
from shape_function import ShapeFunction
from attractor import GoalAttractor

# Goal attractor
# --------------------------------

cs = CanonicalSystem(-2)

time = np.arange(0, np.pi/2, 0.01)
example = np.sin(time)
nbfs = 100

ga = GoalAttractor()
sf = ShapeFunction(cs, nbfs)
sf.train(ga, example, time)

plt.figure()
plt.title('Demo')
plt.plot(time, example,'b')
plt.show()

Fp = sf.responseToTimeArray(time)
plt.figure()
plt.title('Forcing Function: Desired (Fd) and predicted (Fp)')
plt.plot(sf.Fd,'r--', label='Fd')
plt.plot(Fp,'g-', label='Fp')
plt.show()

plt.figure()
plt.title('Weights')
plt.plot(sf.weights,'b')
plt.show()

# Goal attractor
# --------------------------------

from min_jerk_traj import trajectory
  
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
  
ga = GoalAttractor()
ga.setEqPoint(example[-1][0])

x0 = 0
response =  []
for i in xrange(len(example)):
    x1 = example[i][0]
    v1 = example[i][1]
    response.append(ga.response(x1, v1, dt))
    
plt.figure()
plt.title('Goal attractor')
plt.plot(response, 'b')
plt.show()
