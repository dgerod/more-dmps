'''
Copyright (C) 2013 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''
import numpy as np
    
from cs import CanonicalSystem

class TdwFormulation(object):

    def __init__(self, ay=None, by=None):
        
        # Schaal 2012
        if ay is None: ay = 25.
        self.ay = ay
        if by is None: by = ay / 4.
        self.by = by
    
    def acceleration(self, x, dx, start, goal, tau, f, s=None):
 
        ddy = (self.ay *  (self.by * (goal - x) - dx/tau) + f) * tau       
        return ddy
        
    def fs(self, y, dy, ddy, start, goal, tau, s=None):
        
        f_target = ddy - self.ay * (self.by * (goal - y) - dy)
        return f_target
            
class OriginalFormulation(object):

    def __init__(self, K=100.):
        self.K = K
        self.D = self.K/4.
    
    def acceleration(self, x, dx, start, goal, tau, f, s=None):
        return (self.K * (goal - x) - self.D * dx + (goal - start) * f) / tau
  
    def fs(self, y, dy, ddy, start, goal, tau, s=None):
        return ((-1 * self.K * (goal - y) + self.D * dy + tau * ddy) / (goal - start))

class ImprovedFormulation(object):
    
    def __init__(self, K=100.):
        self.K = K
        self.D = self.K/4.
    
    def transformation(self, K, D, x, dx, start, goal, tau, f, s):
        return (K * (goal - x) - D * dx - K * (goal - start) * s + K * f) / tau
    
    def fs(self, K, D, y, dy, ddy, start, goal, tau, s):
        return ((tau**2 * ddy + D * dy * tau) / K ) - (goal - y) + ((goal - start) * s)
    
class DMPs_discrete(object):
   
    """
    An implementation of discrete DMPs
    as described in Dr. Stefan Schaal's (2002) paper.
    """

    def __init__(self, dims, bfs, ts=None, dt=.01,
                 y0=0, goal=1, w=None, 
                 ay=None, by=None, **kwargs):
        """
        dims int: number of dynamic motor primitives
        bfs int: number of basis functions per DMP
        dt float: timestep for simulation
        y0 list: initial state of DMPs
        goal list: goal state of DMPs
        w list: tunable parameters, control amplitude of basis functions
        ay int: gain on attractor term y dynamics
        by int: gain on attractor term y dynamics
        """
        
        # call super class constructor
        # ---

        self.pattern = 'discrete'

        self.dmps = dims 
        self.bfs = bfs 
        self.dt = dt

        # set up the Transformation System
        if ts is None:
            ts =  TdwFormulation()
        self.ts = ts

        # start and goal 
        if isinstance(y0, (int, float)):
            y0 = np.ones(self.dmps)*y0 
        self.y0 = y0
        if isinstance(goal, (int, float)):
            goal = np.ones(self.dmps)*goal
        self.goal = goal 
        
        if w is None: 
            # default is f = 0
            w = np.zeros((self.dmps, self.bfs))
        self.w = w
        
        self.f_desired = np.array([])       
        self.f_predicted = np.array([])       
        self.f = np.zeros(self.dmps)         
        
        if ay is None: ay = np.ones(self.dmps) * 25. # Schaal 2012
        self.ay = ay
        if by is None: by = self.ay.copy() / 4. # Schaal 2012
        self.by = by
        
        # set up the CS 
        self.cs = CanonicalSystem(pattern=self.pattern, dt=self.dt, **kwargs)
        self.timesteps = int(self.cs.run_time / self.dt)

        # set up the DMP system
        self.reset_state()
        
        # ---

        self.prep_centers_and_variances()
        self.check_offset()
        
    def prep_centers_and_variances(self):
        """Set the centre of the Gaussian basis 
        functions be spaced evenly throughout run time"""

        # desired spacings along x
        # need to be spaced evenly between 1 and exp(-ax)
        # lowest number should be only as far as x gets 
        first = np.exp(-self.cs.ax * self.cs.run_time) 
        last = 1.05 - first
        des_c = np.linspace(first, last, self.bfs) 

        self.c = np.ones(len(des_c)) 
        for n in range(len(des_c)): 
            # x = exp(-c), solving for c
            self.c[n] = -np.log(des_c[n])

        # set variance of Gaussian basis functions
        # trial and error to find this spacing
        self.h = np.ones(self.bfs) * self.bfs**1.5 / self.c
        
    def check_offset(self):
        """Check to see if initial position and goal are the same
        if they are, offset slightly so that the forcing term is not 0"""

        for d in range(self.dmps):
            if (self.y0[d] == self.goal[d]):
                self.goal[d] += 1e-4

    def set_goal(self, y_des): 
        """Generate the goal for path imitation. 
        For rhythmic DMPs the goal is the average of the 
        desired trajectory.
    
        y_des np.array: the desired trajectory to follow
        """

        return y_des[:,-1].copy()

    def gen_psi(self, x):
        """Generates the activity of the basis functions for a given 
        state of the canonical system.
        x float: the current state of the canonical system
        """

        if isinstance(x, np.ndarray):
            x = x[:,None]
        return np.exp(-self.h * (x - self.c)**2)

    def gen_front_term(self, x, dmp_num):
        """Generates the diminishing front term on 
        the forcing term.
        x float: the current value of the canonical system
        dmp_num int: the index of the current dmp
        """
        return x * (self.goal[dmp_num] - self.y0[dmp_num])

    def find_force_function(self, dmp_num, y, dy, ddy):
        
        d = dmp_num
          
        #tf = TdwFormulation()
        #f_target = tf.fs(y[d], dy[d], ddy[d], self.y0[d], self.goal[d], 1.0)

        f_target = self.ts.fs(y[d], dy[d], ddy[d], self.y0[d], self.goal[d], 1.0)

        return f_target     

    def calculate_acceleration(self, dmp_num, tau, f, s):
 
        d = dmp_num

        #tf = TdwFormulation()
        #ddy = tf.acceleration(self.y[d], self.dy[d], self.y0[d], self.goal[d], tau, f)

        ddy = self.ts.acceleration(self.y[d], self.dy[d], self.y0[d], self.goal[d], tau, f)

        return ddy
        
    def compute_weights(self, f_target):
        """Generate a set of weights over the basis functions such 
        that the target forcing term trajectory is matched.
        
        f_target np.array: the desired forcing term trajectory
        """

        # calculate x and psi   
        x_track = self.cs.rollout()
        psi_track = self.gen_psi(x_track)

        #efficiently calculate weights for BFs using weighted linear regression
        weights = np.zeros((self.dmps, self.bfs))
        for d in range(self.dmps):
            # spatial scaling term
            k = (self.goal[d] - self.y0[d])
            for b in range(self.bfs):
                numer = np.sum(x_track * psi_track[:,b] * f_target[:,d])
                denom = np.sum(x_track**2 * psi_track[:,b])
                weights[d,b] = numer / (k * denom)
                
        return weights

    def reset_state(self):
        """Reset the system state"""
        self.y = self.y0.copy()
        self.dy = np.zeros(self.dmps)   
        self.ddy = np.zeros(self.dmps)  
        self.cs.reset_state()  

    def step(self, tau=1.0, state_fb=None, external_force=None):
        """Run the DMP system for a single timestep.

        tau float: scales the timestep
                  increase tau to make the system execute faster
        state_fb np.array: optional system feedback
        """

        # run canonical system
        cs_args = {'tau':tau,
                   'error_coupling':1.0}
        if state_fb is not None: 
            # take the 2 norm of the overall error
            state_fb = state_fb.reshape(1,self.dmps)
            dist = np.sqrt(np.sum((state_fb - self.y)**2))
            cs_args['error_coupling'] = 1.0 / (1.0 + 10*dist)
        x = self.cs.step(**cs_args)

        # generate basis function activation
        psi = self.gen_psi(x)

        for d in range(self.dmps):

            # generate the forcing function
            self.f[d] = self.gen_front_term(x, d) * \
                (np.dot(psi, self.w[d])) / np.sum(psi)            

            # DMP acceleration
            self.ddy[d] = self.calculate_acceleration(d, tau, self.f[d], x)

            # Correct acceleration
            if external_force is not None:
                self.ddy[d] += external_force[d]
                        
            self.dy[d] += self.ddy[d] * tau * self.dt * cs_args['error_coupling']
            self.y[d] += self.dy[d] * self.dt * cs_args['error_coupling']

        return self.y, self.dy, self.ddy
        
    def learn(self, y_des):
        """Takes in a desired trajectory and generates the set of 
        system parameters that best realize this path.
    
        y_des list/array: the desired trajectories of each DMP
                          should be shaped [dmps, run_time]
        """

        # set initial state and goal
        # ---

        if y_des.ndim == 1: 
            y_des = y_des.reshape(1,len(y_des))
        self.y0 = y_des[:,0].copy()
        self.y_des = y_des.copy()
        self.goal = self.set_goal(y_des)
        
        self.check_offset()

        # generate function to interpolate the desired trajectory
        # ---

        import scipy.interpolate
        path = np.zeros((self.dmps, self.timesteps))
        x = np.linspace(0, self.cs.run_time, y_des.shape[1])
        for d in range(self.dmps):
            path_gen = scipy.interpolate.interp1d(x, y_des[d])
            for t in range(self.timesteps):  
                path[d, t] = path_gen(t * self.dt)
        y_des = path

        # Calculate velocity and acceleration profiles
        # ---
       
        # calculate velocity of y_des
        dy_des = np.diff(y_des) / self.dt
        # add zero to the beginning of every row
        dy_des = np.hstack((np.zeros((self.dmps, 1)), dy_des))

        # calculate acceleration of y_des
        ddy_des = np.diff(dy_des) / self.dt
        # add zero to the beginning of every row
        ddy_des = np.hstack((np.zeros((self.dmps, 1)), ddy_des))

        # Compute weights
        # ---

        # find the force required to move along this trajectory
        f_desired = np.zeros((y_des.shape[1], self.dmps))
        for d in range(self.dmps):
            f_desired[:,d] = self.find_force_function(d, y_des, dy_des, ddy_des)
        
        # efficiently generate weights to realize f_target
        self.w = self.compute_weights(f_desired)
        self.f_desired = f_desired
        
        self.reset_state()
        self.f_predicted = np.array([])       
        self.f = np.zeros(self.dmps)         

        return y_des

    def plan(self, timesteps=None, **kwargs):
        """Generate a system trial, no feedback is incorporated."""

        self.reset_state()

        if timesteps is None:
            if kwargs.has_key('tau'):
                time_steps = int(self.timesteps / kwargs['tau'])
            else: 
                time_steps = self.timesteps

        # set up tracking vectors
        y_track = np.zeros((time_steps, self.dmps)) 
        dy_track = np.zeros((time_steps, self.dmps))
        ddy_track = np.zeros((time_steps, self.dmps))
        
        f_predicted = np.zeros((time_steps, self.dmps)) 
    
        for t in range(time_steps):
        
            y, dy, ddy = self.step(**kwargs)
            f_predicted[t] = self.f
            
            # record timestep
            y_track[t] = y
            dy_track[t] = dy
            ddy_track[t] = ddy

        self.f_predicted = f_predicted    
        return y_track, dy_track, ddy_track
