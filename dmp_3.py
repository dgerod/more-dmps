
import math

class TransformSystem(object):
 pass

class OriginalFormulation(TransformSystem):

  def transformation(self, K, D, x, raw_xd, start, goal, tau, f, s):
    return (K * (goal - x) - D * raw_xd + (goal - start) * f) / tau
  
  def fs(self, K, D, y, yd, ydd, start, goal, tau, s):
    return ((-1 * K * (goal - y) + D * yd + tau * ydd) / (goal - start))

class ImprovedFormulation(TransformSystem):
  
  def transformation(self, K, D, x, raw_xd, start, goal, tau, f, s):
    return (K * (goal - x) - D * raw_xd - K * (goal - start) * s + K * f) / tau
    
  def fs(self, K, D, y, yd, ydd, start, goal, tau, s):
    # Published formula in the article:
    #   ((tau * ydd - d_gain * yd) / k_gain ) + (goal - y) - ((goal - start) * s)
    # However, we are using version from dmp_lib/transformation_systm.cpp
    return ((tau**2 * ydd + D * yd * tau) / K ) - (goal - y) + ((goal - start) * s)
    
# -----------------------------------------------------------------------------

import numpy as np
from lwr import LWR

class DiscreteDMP(object):

  def __init__(self, transf_system, reg_model=None):

    self.ts = transf_system

    # state variables 
    # -----------------------------
    
    self.start = 0.0
    self.goal = 1.0
    self.tau = 1.0
    
    # Cannonical System
    # -----------------------------
    
    # s_min 
    self.cutoff = 0.001
    
    # alpha (canonical system parameter)
    self.alpha = abs(math.log(self.cutoff))
    
    # Canonical System
    self.s = 1.0 # canonical system is starting with 1.0
    self.s_time = 0.0
    
    # Attractor
    # -----------------------------

    # spring term
    self.K = 50.0
    
    # damping term
    self.D = self.K / 4

    # time steps (for the run)
    self.delta_t = 0.001
    
    # Transformation System
    # ----------------------------
    
    # current position, velocity, acceleration
    
    self.x = 0.0
    self.xd = 0.0
    self.xdd = 0.0 

    # internal variables

    ''' xd not scaled by tau aka v'''
    self._raw_xd = 0.0
    
    self.f = 0
        
    # target function input (x) and output (y)
    self.target_function_input = None
    self.Fd = None
    # debugging (y values predicted by fitted lwr model)
    self.Fp = None
    
    # do not predict f by approximated function, 
    # use values of ft directly only works if duration is 1.0!!
    self.use_ft = False

    # create LWR model and set parameters
    # -----------------------------

    if not reg_model:
      # default regression model
      self.lwr_model = LWR(activation=0.1, exponentially_spaced=True, n_rfs=20)
    else:
      # custom regression model
      self.lwr_model = reg_model       
        
    # is the DMP initialized?
    self._is_learned = False
     
  def _predict_f(self, x):
    
    # if nothing is learned we assume f=0.0
    if not self._is_learned:
      print "WARNING: NO TARGET FUNCTION LEARNED assuming f = 0.0"
      return 0.0
    
    #return self.lwr_model.predict(np.asarray([x]))[0]
    return self.lwr_model.predict(x)

  def _create_training_set(self, trajectory, frequency):
    '''
      Prepares the data set for the supervised learning
      @param trajectory: list of 3-Tuples with (pos,vel,acc) 
    '''
    # number of training samples
    n_samples = len(trajectory)
    
    # duration of the movement
    duration = float(n_samples) / float(frequency)
    
    # set tau to duration for learning
    tau = duration
    
    # initial goal and start obtained from trajectory
    start = trajectory[0][0]
    goal = trajectory[-1][0]
    
    print "create training set of movement from trajectory with %i entries (%i hz) with duration: %f, start: %f, goal: %f" % (n_samples, frequency, duration, start, goal)
    
    # compute target function input (canonical system) [rollout]
    # -----------------------------

    # compute alpha_x such that the canonical system drops
    # below the cutoff when the trajectory has finished

    alpha = -(math.log(self.cutoff))
    
    # time steps
    dt = 1.0 / n_samples # delta_t for learning
    time = 0.0
    time_steps = np.zeros(n_samples)
    for i in xrange(len(time_steps)):
      time_steps[i] = math.exp(-(alpha / tau) * time) 
      time += dt
   
    # compute values of target function
    # -----------------------------
    
    # the target function (transformation system solved for f, and plugged in y for x)
    ft = lambda y, yd, ydd, s : self.ts.fs(self.K, self.D, y, yd, ydd, start, goal, tau, s)
    
    # evaluate function to get the target values for given training data
    F = []
    for i, d in enumerate(trajectory):
      # compute f_target(y, yd, ydd) * s
      #print "s ", target_function_input[i], "y ", d[0], "yd ", d[1], "ydd", d[2], " ft:", ft(d[0], d[1], d[2])
      F.append(ft(d[0], d[1], d[2], time_steps[i]))
    
    return time_steps, np.asarray(F)
  
  def _compute_derivatives(self, pos_trajectory, frequency):
    # ported from trajectory.cpp 
    # no fucking idea why doing it this way - but hey, the results are better ^^
    
    add_pos_points = 4
    #add points to start
    for _ in range(add_pos_points):
      first_point = pos_trajectory[0]
      pos_trajectory.insert(0, first_point)
      
    # add points to the end
    for _ in range(add_pos_points):
      first_point = pos_trajectory[-1]
      pos_trajectory.append(first_point)
      
    # derive positions
    vel_trajectory = []
    
    for i  in range(len(pos_trajectory) - 4):
      vel = (pos_trajectory[i] - (8.0 * pos_trajectory[i + 1]) +
                (8.0 * pos_trajectory[i + 3]) - pos_trajectory[i + 4]) / 12.0
      vel *= frequency
      vel_trajectory.append(vel)
    
    
    # derive velocities
    acc_trajectory = []
    for i  in range(len(vel_trajectory) - 4):     
      acc = (vel_trajectory[i] - (8.0 * vel_trajectory[i + 1]) + 
              (8.0 * vel_trajectory[i + 3]) - vel_trajectory[i + 4]) / 12.0
      acc *= frequency
      acc_trajectory.append(acc)
        
    results = zip(pos_trajectory[4:], vel_trajectory[2:], acc_trajectory)
  
    return results
      
  def _step(self):
    '''
      runs a integration step - updates variables self.x, self.xd, self.xdd
    '''
    
    dt = self.delta_t 
    
    # integrate transformation system   
    # -----------------------------
    
    # update f(s)
    # debugging: use raw function output (time must be 1.0)
    if self.use_ft:
      print "DEBUG: using ft without approximation"
      ftinp = list(self.target_function_input)
      self.f = self.Fd[ftinp.index(self.s)]
    else:
      self.f = self._predict_f(self.s)
    
    # calculate xdd (vd) according to the transformation system equation 1
    self.xdd = self.ts.transformation(self.K, self.D, self.x, self._raw_xd, self.start, self.goal, self.tau, self.f, self.s) 

    # calculate xd using the raw_xd (scale by tau)
    self.xd = (self._raw_xd / self.tau)
    
    # integrate (update xd with xdd)
    self._raw_xd += self.xdd * dt
    
    # integrate (update x with xd) 
    self.x += self.xd * dt
    
    # integrate canonical system
    # -----------------------------
        
    self.s = math.exp(-(self.alpha/self.tau) * self.s_time)
    self.s_time += dt
  
    return self.x, self.xd, self.xdd
    
  def learn(self, trajectory, time):
    '''
     Learns the DMP by a given sample trajectory
     @param sample_trajectory: list of tuples (pos,vel,acc)
    '''
    assert len(trajectory) > 0

    totalSteps = int(time/self.delta_t) # frequency    
    
    if isinstance(trajectory[0], float):
      trajectory = self._compute_derivatives(trajectory, totalSteps)
          
    if len(trajectory[0]) != 3:
      raise Exception("trajectory must be a list with 3-tuples [(1,2,3),(4,5,6)]")
    
    # get input and output of desired target function
    time_steps, Fd = self._create_training_set(trajectory, totalSteps)

    # save input/output of f_target
    self.target_function_input = time_steps
    self.Fd = Fd
    
    # learn LWR Model for this transformation system
    self.lwr_model.learn(time_steps, Fd)
    self._is_learned = True
    
    # debugging: compute learned ft(x)
    self.Fp = []
    for x in time_steps:
      self.Fp.append(self._predict_f(x))      
  
  def setup(self, start, goal, duration):
    self.start = start
    self.goal = goal
    self.tau = duration
  
  def plan(self, time):
      
    trajectory = []
    self.x = self.start
    
    stepNumber = 0
    totalSteps = int(time/self.delta_t)        
    while stepNumber < totalSteps:
      x, xd, xdd = self._step()
      trajectory.append([x, xd, xdd])
      stepNumber += 1
   
    pos = np.squeeze(np.array(np.matrix(trajectory)[:,0]))
    vel = np.squeeze(np.array(np.matrix(trajectory)[:,1]))
    acc = np.squeeze(np.array(np.matrix(trajectory)[:,2]))
    return pos, vel, acc
