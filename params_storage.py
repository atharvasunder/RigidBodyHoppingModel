# File for storing params classes for bodies, muscles, joints, nervous system 
import math
import numpy as np

from dataclasses import dataclass

# @dataclass
# class general_params:
#     # integration params
#     dt: float  # (s) integration time step
#     tStop: float  # (s) simulation stop time

@dataclass
class muscle_params:
    # MTC properties
    l_ref_mtc: float # (m) reference length for calculation of q
    q_ref_mtc: float # (deg) reference angle for calculation of q
    d: float # (m) moment arm of MTC

    # Force-length relationship
    F_max: float # (N) max force MTC can produce
    l_opt: float # (m) peak of force-length relationship
    w: float # param for the gaussian shaped F-l curve
    c : float # param for the gaussian shaped F-l curve

    # Force-velocity relationship
    v_max: float # (m/s) max shortening velocity of muscle
    N: float # eccentric force enhancement
    K: float # curvature constant

    # Tendon properties
    l_rest: float # (m) rest length of tendon
    e_ref_see: float # reference strain for tendon force calculation

@dataclass
class nervous_params:
    # Activation parameters
    tau: float  # (s) excitation contraction coupling constant
    delP: float  # (s) feedback time delay

    # Parameters for feedback-based stimulation
    stim0: float # Constant stimulation received by muscle
    G: float # gain for force feedback
    l_off: float # offset for length feedback