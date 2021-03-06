
import numpy as np  
from cs import CanonicalSystem

class TdwFormulation(object):

    def __init__(self, ay=None, by=None):
        # Parameters as defined in Schaal 2012
        if ay is None: ay = 25.
        self.ay = ay
        if by is None: by = ay/4.
        self.by = by
    
    def acceleration(self, x, dx, start, goal, tau, f, s):
        return (self.ay *  (self.by * (goal-x) - dx/tau) + (goal-start)*f) * tau       
            
    def fs(self, x, dx, ddx, start, goal, tau, s):        
        return (ddx - self.ay * (self.by * (goal-x) - dx) / (goal-start)) 
        
class OriginalFormulation(object):

    def __init__(self, K=100., D=None):
        self.K = K
        if D is None: 
            D = 2.0 * np.sqrt(self.K)    
        self.D = D
            
    def acceleration(self, x, dx, start, goal, tau, f, s):
        return (self.K*(goal-x) - self.D*dx + (goal-start)*f) / tau
  
    def fs(self, x, dx, ddx, start, goal, tau, s):
        return (tau*ddx - self.K*(goal-x) + self.D*dx) / (goal-start)
        
class ImprovedFormulation(object):
    
    def __init__(self, K=100., D=None):
        self.K = K
        if D is None: 
            D = 2.0 * np.sqrt(self.K)    
        self.D = D
    
    def acceleration(self, x, dx, start, goal, tau, f, s):
        return (self.K*(goal-x) - self.D*dx - self.K*(goal-start)*s + self.K*f) / tau
    
    def fs(self, x, dx, ddx, start, goal, tau, s):
        return ((tau**2*ddx + self.D*dx*tau) / self.K) - (goal-x) + (goal-start)*s
  
# -----------------------------------------------------------------------------
  
class DMPs_discrete(object):

    def __init__(self, dims, bfs, ts=None, dt=.01,
                 y0=0, goal=1, w=None, 
                 ay=None, by=None, **kwargs):
        '''
        dims int: number of dynamic motor primitives
        bfs int: number of basis functions per DMP
        dt float: timestep for simulation
        y0 list: initial state of DMPs
        goal list: goal state of DMPs
        w list: tunable parameters, control amplitude of basis functions
        ay int: gain on attractor term y dynamics
        by int: gain on attractor term y dynamics
        '''
        
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
        
        # set up the CS 
        self.cs = CanonicalSystem(pattern='discrete', dt=self.dt, **kwargs)
        self.time_steps = int(self.cs.run_time / self.dt)

        # set up the DMP system
        self.reset_state()
   
        self.prep_centers_and_variances()
        self.check_offset()
        
    def prep_centers_and_variances(self):
        '''
        Set the centre of the Gaussian basis functions be spaced evenly 
        throughout run time.
        '''

        # desired spacings along x
        # need to be spaced evenly between 1 and exp(-ax)
        # lowest number should be only as far as x gets 
        first = np.exp(-self.cs.ax * self.cs.run_time) 
        last = 1.05 - first
        des_c = np.linspace(first, last, self.bfs) 

        self.c = np.ones(len(des_c)) 
        for n in range(len(des_c)): 
            self.c[n] = -np.log(des_c[n])

        # set variance of Gaussian basis functions
        # trial and error to find this spacing
        self.h = np.ones(self.bfs) * self.bfs**1.5 / self.c
        
    def check_offset(self):
        '''
        Check to see if initial position and goal are the same
        if they are, offset slightly so that the forcing term is not 0.
        '''

        for idx in range(self.dmps):
            if (self.y0[idx] == self.goal[idx]):
                self.goal[idx] += 1e-4

    def set_goal(self, y_des): 
        '''
        Generate the goal for path imitation. For rhythmic DMPs the goal is the 
        average of the desired trajectory.
    
        y_des np.array: the desired trajectory to follow
        '''

        return y_des[:,-1].copy()

    def gen_psi(self, x):
        '''
        Generates the activity of the basis functions for a given state of the 
        canonical system.
        
        x float: the current state of the canonical system
        '''

        if isinstance(x, np.ndarray):
            x = x[:,None]
        return np.exp(-self.h * (x - self.c)**2)        

    def gen_forcing_term(self, x, dmp_num, scale=False):
        """Calculates the complete forcing term .

        x float: the current value of the canonical system (s)
        dmp_num int: the index of the current dmp
        scaled bool: apply scalation (g-y0) to the forcing term (T/F)
        """
        
        psi = self.gen_psi(x)
        f = x * ((np.dot(psi, self.w[dmp_num])) / np.sum(psi))
        if scale:
            f = f * (self.goal[dmp_num] - self.y0[dmp_num])
        return f
                
    def find_force_function(self, dmp_num, y, dy, ddy, s):
        
        d = dmp_num
        f_target = self.ts.fs(y[d], dy[d], ddy[d], self.y0[d], self.goal[d], 1.0, s)
        return f_target     

    def calculate_acceleration(self, dmp_num, tau, f, s):
 
        d = dmp_num
        ddy = self.ts.acceleration(self.y[d], self.dy[d], self.y0[d], self.goal[d], tau, f, s)
        return ddy
        
    def compute_weights(self, f_target):
        '''
        Generate a set of weights over the basis functions such that the target forcing 
        term trajectory is matched.
        
        f_target np.array: the desired forcing term 
        '''
        
        # calculate x/s and psi(s)
        x_track = self.cs.rollout()
        psi_track = self.gen_psi(x_track)

        # efficiently calculate weights for BFs using weighted linear regression
        self.w = np.zeros((self.dmps, self.bfs))
        for d in range(self.dmps):
            for b in range(self.bfs):
                numer = np.sum(x_track    * psi_track[:,b] * f_target[:,d])
                denom = np.sum(x_track**2 * psi_track[:,b])
                self.w[d,b] = numer / denom
        
        return (x_track, psi_track)

    def reset_state(self):
        '''
        Reset the system state
        '''
        self.y = self.y0.copy()
        self.dy = np.zeros(self.dmps)   
        self.ddy = np.zeros(self.dmps)  
        self.cs.reset_state()  

    def step(self, tau=1.0, state_fb=None, external_force=None):
        '''
        Run the DMP system for a single timestep.
        
        tau float: scales the timestep
                   increase tau to make the system execute faster
        state_fb np.array: optional system feedback
        '''

        # run canonical system
        cs_args = {'tau': tau, 'error_coupling': 1.0}
                   
        if state_fb is not None: 
            # take the 2 norm of the overall error
            state_fb = state_fb.reshape(1,self.dmps)
            dist = np.sqrt(np.sum((state_fb - self.y)**2))
            cs_args['error_coupling'] = 1.0 / (1.0 + 10*dist)
        x = self.cs.step(**cs_args)

        for idx in range(self.dmps):

            # Calcualte acceleration based on f(s)
            self.f[idx] = self.gen_forcing_term(x, idx)            
            self.ddy[idx] = self.calculate_acceleration(idx, tau, self.f[idx], x)
            
            # Correct acceleration
            if external_force is not None:
                self.ddy[idx] += external_force[idx]
                        
            self.dy[idx] += self.ddy[idx] * tau * self.dt * cs_args['error_coupling']
            self.y[idx] += self.dy[idx] * self.dt * cs_args['error_coupling']

        return self.y, self.dy, self.ddy
        
    def learn(self, y_des):
        '''
        Takes in a desired trajectory and generates the set of system parameters 
        that best realize this path.
    
        y_des list/array: the desired trajectories of each DMP
                          should be shaped [dmps, run_time]
        '''

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
        path = np.zeros((self.dmps, self.time_steps))
        x = np.linspace(0, self.cs.run_time, y_des.shape[1])
        for idx in range(self.dmps):
            path_gen = scipy.interpolate.interp1d(x, y_des[idx])
            for t in range(self.time_steps):  
                path[idx, t] = path_gen(t * self.dt)
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
        
        # Compute F and weights
        # ---

        # run canonical system
        x = self.cs.rollout()
        
        # find the force required to move along this trajectory
        f_desired = np.zeros((y_des.shape[1], self.dmps))
        for idx in range(self.dmps):
            f_desired[:,idx] = self.find_force_function(idx, y_des, dy_des, ddy_des, x)
        
        # efficiently generate weights to realize f_target
        self.x_track, self.psi_track = self.compute_weights(f_desired)
        self.f_desired = f_desired
        
        self.reset_state()
        return y_des, dy_des, ddy_des

    def plan(self, time_steps=None, **kwargs):
        '''
        Generate a system trial, no feedback is incorporated.
        '''

        self.reset_state()

        if time_steps is None:
            if kwargs.has_key('tau'):
                time_steps = int(self.time_steps / kwargs['tau'])
            else: 
                time_steps = self.time_steps

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
