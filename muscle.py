import math
import numpy as np
from fifo_buffer import FIFOBuffer


class muscle_tendon_unit():
    """
    Class describing a muscle connected between two bodies. 

    In: 
    base_body, mate_body: the two bodies between which the
                          muscle is located

    base_site, mate_site: the specific connection sites at these bodies

    muscle_params:
    - MTC properties
    - Force-length relationship params
    - Force-velocity relationship params
    - Tendon properties
    
    Out: n/a

    """

    # constructor
    def __init__(self, name, base_body, base_muscle_site, mate_body, mate_muscle_site, muscle_params, nervous_params,dt):
        self.name = name
        self.base = base_muscle_site            # base site of muscle
        self.mate = mate_muscle_site            # mate site of muscle
        self.base_body = base_body  # body to which base site belongs
        self.mate_body = mate_body  # body to which mate site belongs

        pi = math.pi

        self.tau_contribution = 0.0    # joint torque, initialized as 0
        self.q = pi - abs(self.base_body.p - self.mate_body.p)  # joint angle

        self.muscle_params = muscle_params
        self.nervous_params = nervous_params

        # Tendon properties
        self.l_rest = muscle_params.l_rest
        self.e_ref_see = muscle_params.e_ref_see

        # Force-length relationship
        self.F_max = muscle_params.F_max
        self.l_opt = muscle_params.l_opt
        self.c = muscle_params.c
        self.w = muscle_params.w

        # Force-velocity relationship
        self.K = muscle_params.K
        self.N = muscle_params.N
        self.v_max = muscle_params.v_max

        # MTC properties
        self.l_ref_mtc = muscle_params.l_ref_mtc
        self.q_ref_mtc = muscle_params.q_ref_mtc
        self.d = muscle_params.d

        # nervous system properties
        # Optimized parameters for feedback-based stimulation
        self.stim0 = nervous_params.stim0  # Constant stimulation from the brain to the muscle
        self.tau = nervous_params.tau # activation dynamics constantr
        self.delP = nervous_params.delP
        # Gain for force feedback
        self.G = nervous_params.G
        self.l_off = nervous_params.l_off

        # muscle activation
        self.A = 0.001 # initialized as 0.001
        self.A_list = [self.A]
        
        # initialize tendon length to rest length
        self.l_se = self.l_rest
        self.l_ce = 0
        

        self.F_mtc = None # not relevant in flight rn
        self.F_mtc_list = [self.F_mtc] # just for plotting

        self.f_l = None    # zero force at flight phase
        self.f_l_list = [self.f_l]

        self.f_v = None # zero velocity at flight phase
        self.f_v_list = [self.f_v]
        
        self.buffer_size = int(self.delP/dt)
        self.neural_delay = FIFOBuffer(self.buffer_size)
    
    def update(self, dt):
        # function keeps updating mtc force based on instantaneous activation

        # compute l_mtc from system geometry 
        # initialize l_ce, Activation while defining the muscle
        # everytime update is called, get l_se,
        # use it to compute F_mtc
        # use l_ce to get f_l,
        # Use F_mtc to compute brain stimulation and activation derivative.
        # Integrate activation derivative to get current activation
        # compute f_v with
        # f_l, A, F_mtc, pass it to inverse function
        # to get v_ce 
        # integrate v_ce to get l_ce new, need dt, l_ce at previous timestep    
        # replace self.l_ce with the updated one
        # replace self.A with the current one
        # return F_mtc/ use F_mtc to compute torques and return torques
        
        pi = math.pi

        q = pi - abs(self.base_body.p - self.mate_body.p)  # joint angle between segments, as defined in the paper
        A = self.A
        l_ce = self.l_ce

        # l_mtc instantaneous length
        l_mtc = self.l_ref_mtc - self.d*(q - self.q_ref_mtc)
        # print(l_mtc)

        if l_ce == 0:  # initial condition
            l_se = self.l_rest
            l_ce = l_mtc - l_se
            self.l_ce = l_ce
        else:
            l_se = l_mtc - self.l_ce # length of series elastic element

        # series elastic element force (= mtc force)
        F_mtc = self.F_max*self.f_se(l_se/self.l_rest , self.e_ref_see)
        self.F_mtc = F_mtc
        # print(F_mtc)
        
        l_ce_norm = self.l_ce/self.l_opt
        # force length relationship, calculated using l_ce
        f_l = self.f_l_relationship(l_ce_norm, self.c, self.w) # Force length relationship of contractile element
        self.f_l = f_l

        # Force velocity relationship
        f_v = F_mtc/(self.F_max*f_l*A + 1e-15)
        self.f_v = f_v

        # apply inverse of force velocity relationship
        v_ce = self.f_v_inverse(f_v, self.K, self.N)
        # print(v_ce)
        
        # apply Feedback transport delay
        F_mtc_feedback = self.neural_delay.push(F_mtc) # returns none unless the buffer is full
        
        # calculate brain stimulation (control to the activation dynamics)
        S = F_mtc_feedback*self.G + self.stim0

        # update all variables of this class

        # integrate using forward euler to get l_ce for next timestep
        self.l_ce = l_ce + v_ce*dt # (not sure about rk4 here)
        self.q = q
        # to be updated with params
        self.A = np.clip(A + self.activation_dynamics(A, S, self.tau) * dt, 0, 1)

        # forces on each site (remember that muscle sites are different from joint sites!!!)
        self.base.fx = 0
        self.base.fz = -0*F_mtc
        self.base.tau = -self.d*F_mtc # always clockwise rotation

        self.mate.fx = 0*F_mtc
        self.mate.fz = 0
        self.mate.tau = self.d*F_mtc # always anticlockwise rotation
        
        # print("muscle torque contribution = ", self.d*F_mtc)

        self.tau_contribution = self.d*F_mtc # just for logging

        # return
    
    def reset(self):

        self.A = 0.001
        self.l_se = self.l_rest
        self.l_ce = 0
        
        self.F_mtc = None # not relevant in flight rn
        self.f_l = None    # zero force at flight phase
        self.f_v = None # zero velocity at flight phase
        
        self.base.tau = 0
        self.mate.tau = 0

    def f_se(self, l_se_norm, e_ref_see):

        e = l_se_norm - 1

        if e > 0:
            return (e / e_ref_see) ** 2
        else:
            return 0

    def f_l_relationship(self, l_ce_norm, c, w):

        return math.exp(c * (abs((l_ce_norm - 1) / w) ** 3))
    
    def f_v_inverse(self, f_v, K, N):

        if f_v < 1:
            v_ce = (f_v - 1) / (1 + K * f_v)
        elif 1 <= f_v < N:
            v_ce = (1 - f_v) / ((1 - N) + 7.56 * K * (f_v - N))
        elif f_v >= N:
            v_ce = 1
        else:
            v_ce = 0

        return v_ce
    
    def activation_dynamics(self, A, S, tau):

        return (1/tau)*(S - A)