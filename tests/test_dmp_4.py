# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pylab as plt

from dmp_4 import DMPs_discrete

# 
# -------------------------------

dmp = DMPs_discrete(dims=1, bfs=10, w=np.zeros((1,10)))
y_track, dy_track, ddy_track = dmp.plan()

plt.figure(1, figsize=(6,3))
plt.plot(np.ones(len(y_track))*dmp.goal, 'r--', lw=2)
plt.plot(y_track, lw=2)
plt.title('DMP system - no forcing term')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['goal', 'system state'], loc='lower right')
plt.tight_layout()

# 
# -------------------------------

# test imitation of path run
plt.figure(2, figsize=(6,4))
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
    
    plt.figure(2)
    plt.subplot(211)
    plt.plot(y_track[:,0], lw=2)
    plt.subplot(212)
    plt.plot(y_track[:,1], lw=2)
    
plt.subplot(211)
a = plt.plot(path1 / path1[-1] * dmp.goal[0], 'r--', lw=2)
plt.title('DMP imitate path')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend([a[0]], ['desired path'], loc='lower right')
plt.subplot(212)
b = plt.plot(path2 / path2[-1] * dmp.goal[1], 'r--', lw=2)
plt.title('DMP imitate path')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['%i BFs'%i for i in num_bfs], loc='lower right')

plt.tight_layout()
plt.show()

# 
# -------------------------------

from dmp_4 import TdwFormulation, OriginalFormulation

# test imitation of path run
plt.figure(3, figsize=(6,4))

# a straight line to target
path1 = np.sin(np.arange(0,1,.01)*5)
# a strange path to target
path2 = np.zeros(path1.shape)
path2[int(len(path2) / 2.):] = .5 

forms = [TdwFormulation(), OriginalFormulation()]
name_forms = ["tdw", "original"]

num_bfs = 100

for ii, ts in enumerate(forms):
    
    print ts
    dmp = DMPs_discrete(dims=2, bfs=num_bfs, ts=ts)
    dmp.learn(y_des=np.array([path1, path2]))
    
    # change the goal position
    dmp.goal[0] = 3; dmp.goal[1] = 2          
    y_track, dy_track, ddy_track = dmp.plan()
    
    plt.figure(2)
    plt.subplot(211)
    plt.plot(y_track[:,0], lw=2)
    plt.subplot(212)
    plt.plot(y_track[:,1], lw=2)

plt.subplot(211)

a = plt.plot(path1 / path1[-1] * dmp.goal[0], 'r--', lw=2)
plt.title('DMP imitate path')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend([a[0]], ['desired path'], loc='lower right')
plt.subplot(212)

b = plt.plot(path2 / path2[-1] * dmp.goal[1], 'r--', lw=2)
plt.title('DMP imitate path')
plt.xlabel('time (ms)')
plt.ylabel('system trajectory')
plt.legend(['%s ' % i for i in name_forms], loc='lower right')

plt.tight_layout()
plt.show()
