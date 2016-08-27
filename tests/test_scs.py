import numpy as np
import matplotlib.pyplot as plt
from simple_cs import CanonicalSystem

time = np.arange(0, np.pi/2, 0.01)
num_samples = 10

finalTime = time[-1]
increment = finalTime/(num_samples)

t = np.zeros([(num_samples)])
s = np.zeros([(num_samples)])
    
cs = CanonicalSystem(-2)
for i in xrange(num_samples):
    t[i] = i*increment
    s[i] = cs.response(t[i])
  
plt.figure()
plt.title('Canonical system - t vs. s')
plt.plot(t, t[::-1], 'r-')
plt.plot(t, s, 'g-')
plt.show()
