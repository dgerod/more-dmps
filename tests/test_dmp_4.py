# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pylab as plt

from dmp_4 import DMPs_discrete
from dmp_4 import TdwFormulation, OriginalFormulation

# 
# -------------------------------

dmp1 = DMPs_discrete(dims=1, bfs=10, w=np.zeros((1,10)), ts=TdwFormulation())
y1_track, dy1_track, ddy1_track = dmp1.plan()

plt.figure(1, figsize=(6,3))
plt.plot(np.ones(len(y1_track))*dmp1.goal, 'r--', lw=2)
plt.plot(np.ones(len(y1_track))*dmp1.y0, 'g--', lw=2)
plt.plot(y1_track, 'b-', lw=2)
plt.title('DMP system - no forcing term')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['start', 'goal', 'system state'], loc='lower right')
plt.tight_layout()

dmp2 = DMPs_discrete(dims=1, bfs=10, w=np.zeros((1,10)), ts=OriginalFormulation())
y2_track, dy2_track, ddy2_track = dmp2.plan()

plt.figure(2, figsize=(6,3))
plt.plot(np.ones(len(y2_track))*dmp2.goal, 'r--', lw=2)
plt.plot(np.ones(len(y2_track))*dmp2.y0, 'g--', lw=2)
plt.plot(y2_track, 'b-', lw=2)
plt.title('DMP system - no forcing term')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['start', 'goal', 'system state'], loc='lower right')
plt.tight_layout()

# 
# -------------------------------

# test imitation of path run
plt.figure(3, figsize=(6,4))
num_bfs = [10, 30, 50, 100, 10000]

# a straight line to target
path1 = np.sin(np.arange(0,1,.01)*5)
# a strange path to target
path2 = np.zeros(path1.shape)
path2[int(len(path2) / 2.):] = .5 

for ii, bfs in enumerate(num_bfs):
    
    dmp = DMPs_discrete(dims=2, bfs=bfs)    
    dmp.learn(y_des=np.array([path1, path2]))
    
    # change the goal position
    dmp.goal[0] = 3; dmp.goal[1] = 2          
    y_track, dy_track, ddy_track = dmp.plan()
    
    plt.subplot(211)
    plt.plot(y_track[:,0], lw=2)
    plt.subplot(212)
    plt.plot(y_track[:,1], lw=2)
    
plt.subplot(211)
a = plt.plot(path1 / path1[-1] * dmp.goal[0], 'r--', lw=2)
plt.title('DMP imitate path (X)')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend([a[0]], ['desired path'], loc='lower right')

plt.subplot(212)
b = plt.plot(path2 / path2[-1] * dmp.goal[1], 'r--', lw=2)
plt.title('DMP imitate path (Y)')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['%i BFs'%i for i in num_bfs], loc='lower right')

plt.tight_layout()
plt.show()

# 
# -------------------------------

from dmp_4 import TdwFormulation, OriginalFormulation

# test imitation of path run
plt.figure(4, figsize=(6,4))

# a straight line to target
path1 = np.sin(np.arange(0,1,.01)*5)
# a strange path to target
path2 = np.zeros(path1.shape)
path2[int(len(path2) / 2.):] = .5 

forms = [TdwFormulation(), OriginalFormulation()]
name_forms = ["tdw", "original"]

num_bfs = 100
dmps =  []

for ii, ts in enumerate(forms):
    
    print ts
    dmp = DMPs_discrete(dims=2, bfs=num_bfs, ts=ts)
    dmp.learn(y_des=np.array([path1, path2]))
        
    # change the goal position
    dmp.goal[0] = 3; dmp.goal[1] = 2          
    y_track, dy_track, ddy_track = dmp.plan()
    
    plt.subplot(211)
    plt.plot(y_track[:,0], lw=2)
    plt.subplot(212)
    plt.plot(y_track[:,1], lw=2)

    dmps.append(dmp)
    
plt.subplot(211)
a = plt.plot(path1 / path1[-1] * dmp.goal[0], 'r--', lw=2)
plt.title('DMP imitate path (X)')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend([a[0]], ['desired path'], loc='lower right')

plt.subplot(212)
b = plt.plot(path2 / path2[-1] * dmp.goal[1], 'r--', lw=2)
plt.title('DMP imitate path (Y)')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['%s ' % i for i in name_forms], loc='lower right')

plt.tight_layout()
plt.show()
