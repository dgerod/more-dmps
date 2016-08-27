
from cs import CanonicalSystem
import numpy as np
import matplotlib.pyplot as plt

# Discrete
# ---------------------------------------------
    
cs = CanonicalSystem(dt=.001, pattern='discrete')

# test normal rollout
x_track1 = cs.rollout()

cs.reset_state()

# test error coupling
timesteps = int(1.0/.001)
x_track2 = np.zeros(timesteps)
err = np.zeros(timesteps)
err[200:400] = 2
err_coup = 1.0 / (1 + err)
for i in range(timesteps):
    x_track2[i] = cs.step(error_coupling=err_coup[i])

# plot results
fig, ax1 = plt.subplots(figsize=(6,3))
ax1.plot(x_track1, lw=2)
ax1.plot(x_track2, lw=2)
plt.grid()
plt.legend(['normal rollout', 'error coupling'])
ax2 = ax1.twinx()
ax2.plot(err, 'r-', lw=2)
plt.legend(['error'], loc='lower right')
plt.ylim(0, 3.5)
plt.xlabel('time (s)')
plt.ylabel('x')
plt.title('Canonical system - discrete')

for t1 in ax2.get_yticklabels():
    t1.set_color('r')

plt.tight_layout()

# Rhytmic
# ---------------------------------------------

cs = CanonicalSystem(dt=.001, pattern='rhythmic')

# test normal rollout
x_track1 = cs.rollout()

# plot results
fig, ax1 = plt.subplots(figsize=(6,3))
ax1.plot(x_track1, lw=2)
plt.grid()
plt.legend(['normal rollout'], loc='lower right')
plt.xlabel('time (s)')
plt.ylabel('x')
plt.title('Canonical system - rhythmic')
plt.show()

