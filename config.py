"""
Rigid Body System Configuration

This configuration file defines a rigid body system and assigns all
relevant kinematic and dynamic properties. The properties need to be
configured by the user in this file.

"""

import math
from body_system import RigidBodySystem
from interaction.revolute import RevoluteJoint
from interaction.contact import GroundContact
from params_storage import muscle_params, nervous_params
from muscle import muscle_tendon_unit

# transforms coordinates of point in frame B to frame A
# rA = rAB + Cab*rB
def transform_point(pos_ab, angle_ab, pos_b):
    cp, sp = math.cos(angle_ab), math.sin(angle_ab)
    x_a = pos_ab[0] + cp * pos_b[0] - sp * pos_b[1]
    z_a = pos_ab[1] + sp * pos_b[0] + cp * pos_b[1]

    return x_a, z_a

def human_model(dt):

    pi = math.pi

    # parameters
    NAME = 'Human Hopping Model'

    mass = 1 # mass of one segment

    DAMPING = 4000  # can go down to 2000 and then have dt=3e-4, spring damper constraint force damping coefficient
    STIFFNESS = 0.1 * DAMPING**2 / (4*mass) # stiffness corresponding to critical damping as we dont want oscillations
    print("joint stiffness = ", STIFFNESS)
    k_lim = 10 * 180/pi;    # [Nm/rad] mechanical limits stiffness
    q_dot_max = 0.01 * pi/180;  # [rad/s] mechanical limits damping

    x0 = 0  # inital forward (along horizontal axis) position of trunk CoM
    h0 = 1.18 # initial height (along vertical axis)

    initial_trunk_lean = 0 * pi / 180  # This angle is wrt global frame (not a joint angle!). 
                                        # note that negative angle is clockwise, measured from x-axis of global to x_tr
    initial_hip_angle =  8.1* pi / 180  # all other angles are the joint angles, defined in the folowing way:
    initial_knee_angle = -16.2 * pi / 180  # eg, thigh.p - trunk.p = initial_hip_angle (hip angle = p_mate - p_base)
    # initial_ankle_angle = 90 * pi / 180 # here mate is the body that rotates when the joint is moved

    # joint angle limits
    q_max_hip = 110*pi/180
    q_min_hip = 8.1*pi/180 # slack provided as tight joint limits are enforced

    q_max_knee = -16.2*pi/180
    q_min_knee = -150*pi/180

    # q_max_ankle = 120*pi/180
    # q_min_ankle = 0.01

    # q_max_hip = 35*pi/180  # tight joints, just used for testing.
    # q_min_hip = 25*pi/180

    # q_max_knee = -40*pi/180
    # q_min_knee = -50*pi/180

    # q_max_ankle = 95*pi/180
    # q_min_ankle = 85*pi/180

    # create empty rigid body system
    rbs = RigidBodySystem(NAME)

    # add segments
    # (2x normal values for leg segment masses to account for two legs)
    trunk = rbs.add_body('trunk', mass=78, moment_of_inertia=5000,
                         x=0.0 + x0, z=0.0 + h0, p=0.0 * pi / 180,
                         vx=0.0, vz=0.0, vp=0.0)
    trunk.add_site('hip', x_b=0.0, z_b=0.0) # single site connecting to the hip at x = 0, z = 0 in trunk body frame

    thigh = rbs.add_body('thigh',
                         mass=mass, moment_of_inertia=0.05,
                         x=0.0 + x0, z=-0.05 + h0, p=0.0 * pi / 180,
                         vx=0.0, vz=0.0, vp=0.0)
    thigh.add_site('hip', x_b=0.0, z_b=0.25) # note that the connection to the hip is defined along +ve z axis of the thigh body!
    thigh.add_site('knee', x_b=0.0, z_b=-0.25)
    thigh.add_site('muscle base', x_b=0.04, z_b=0.20) # moment arm on base is smaller than that of mate (diagram in notebook)

    shank = rbs.add_body('shank', mass=mass, moment_of_inertia=0.05,
                         x=0.0 + x0, z=-0.05 + h0, p=0.0 * pi / 180,
                         vx=0.0, vz=0.0, vp=0.0)
    shank.add_site('knee', x_b=0.0, z_b=0.25) # note that the connection to the knee is defined along +ve z axis of the shank body!
    # shank.add_site('ankle', x_b=0.0, z_b=-0.25)
    shank.add_site('ball', x_b=0.0, z_b=-0.25)
    shank.add_site('muscle mate', x_b=0.0, z_b=0.21) # ensures that moment arm is = d (0.04)

    # foot = rbs.add_body('foot', mass=1.9, moment_of_inertia=0.01,
    #                     x=0.0 + x0, z=-0.225 + h0, p=0.0,
    #                     vx=0.0, vz=0.0, vp=0.0)
    # foot.add_site('ankle', x_b=0.0, z_b=0.075) # note that the connection to the ankle is defined along +ve z axis of the foot!
    # foot.add_site('ball', x_b=0.0, z_b=-0.125)

    # add body geometries for animation
    trunk.geometry = ((-0.08, 0.08, 0.08, -0.08, -0.08), # The first tuple eg:(-0.04, 0.04, 0.04, -0.04, -0.04) represents the x-coordinates.
                      (-0.25, -0.25, 0.25, 0.25, -0.25))      # The second tuple eg:(-0.3, -0.3, 0.3, 0.3, -0.3) represents the z-coordinates.
    thigh.geometry = ((-0.03, 0.03, 0.03, -0.03, -0.03), # these form a rectangular shape, Each shape is a closed loop, 
                      (-0.25, -0.25, 0.25, 0.25, -0.25))  # where the last point repeats the first point to ensure proper drawing.
    shank.geometry = ((-0.03, 0.03, 0.03, -0.03, -0.03),
                      (-0.25, -0.25, 0.25, 0.25, -0.25))
    # foot.geometry = ((-0.01,  0.01, 0.01, -0.01, -0.01),
    #                  (-0.125, -0.125, 0.125, 0.125, -0.125))

    # apply initial conditions
    trunk.p = initial_trunk_lean # once initial orientation is defined, we update site coordinates to get initial site coordinates in world frame
    trunk.update_site_coords()

    # we set the initial positions of each link/body here, as it all depends on the previous body's position and the current bodys orientation 
    # wrt the prev body and cannot be defined independently. The initializations above are not used
    # We start by defining the global position and orientation of the shank, and then define positions and orientations of
    # each link below the shank sequentially
    thigh.p = trunk.p + initial_hip_angle # angle made by mate thigh with world frame
    # print(thigh.p)
    pos_ab = (trunk.x + trunk.sites[0].x_b2w, # the [x_b2w,z_b2w] is the vector from the center of the trunk to the trunk-thigh site expresed in the world frame
              trunk.z + trunk.sites[0].z_b2w) # position of of trunk-thigh site wrt world frame origin
    thigh.x, thigh.z = transform_point(pos_ab, thigh.p, (0.0, -0.25)) # gives the cordinates of the com of thigh in world frame
    thigh.update_site_coords()

    shank.p = thigh.p + initial_knee_angle
    # print(shank.p)
    pos_ab = (thigh.x + thigh.sites[1].x_b2w,
              thigh.z + thigh.sites[1].z_b2w)
    shank.x, shank.z = transform_point(pos_ab, shank.p, (0.0, -0.25))
    shank.update_site_coords()

    # foot.p = shank.p + initial_ankle_angle
    # pos_ab = (shank.x + shank.sites[1].x_b2w,
    #           shank.z + shank.sites[1].z_b2w)
    # foot.x, foot.z = transform_point(pos_ab, foot.p, (0.0, -0.075))
    # foot.update_site_coords()

    # create joints, which ensure that the bodies stay connected.
    hip = RevoluteJoint('hip', trunk, trunk.sites[0], thigh, thigh.sites[0], 
                        q_min_hip, q_max_hip, q_dot_max,  # min and max joint angles defined intuitively based on geometry
                        STIFFNESS, DAMPING, k_lim)
    knee = RevoluteJoint('knee', thigh, thigh.sites[1], shank, shank.sites[0], 
                         q_min_knee, q_max_knee, q_dot_max,
                         STIFFNESS, DAMPING, k_lim)
    # ankle = RevoluteJoint('ankle', shank, shank.sites[1], foot, foot.sites[0], 
    #                       q_min_ankle, q_max_ankle, q_dot_max,
    #                       STIFFNESS, DAMPING, k_lim)
    rbs.joint_list = [hip,knee]

    # create ground contact points
    # ball = GroundContact('ball contact', shank, shank.sites[1],
    #                       stiffness_x=4000.0, max_vx=0.1,
    #                       stiffness_z=80000.0, max_vz=0.1,
    #                       mu_slide=0.6, v_transition=0.01,
    #                       mu_stick=0.8)
    
    ball = GroundContact('ball contact', shank, shank.sites[1],
                          stiffness_x=8000.0, max_vx=0.05,
                          stiffness_z=80000.0, max_vz=0.1,
                          mu_slide=0.99, v_transition=0.1,
                          mu_stick=0.99)
    
    rbs.contact_list = [ball]

    # creating muscle tendon units

    F_max=22000
    q_ref_mtc= 110*pi/180 # landing configuration should not have any muscle force
    # i.e. the see is assumed to start at rest length, ce can be compressed
    # and activation is equal to 0.

    knee_extensor_muscle_params = muscle_params(l_ref_mtc=0.5, q_ref_mtc=q_ref_mtc, d=0.04, F_max=F_max, l_opt=0.1, w=0.4, c=math.log(0.05),
                                       v_max=1.2, N=1.5, K=5, l_rest=0.4, e_ref_see=0.04)
    
    neural_params = nervous_params(tau=0.01, delP=0.015, stim0=0.03, G=2.94/F_max, l_off=0.08)

    jump_muscle = muscle_tendon_unit('knee muscle', thigh, thigh.sites[2], shank, shank.sites[2], knee_extensor_muscle_params, neural_params,dt)
    rbs.muscle_list = [jump_muscle]  # connects the muscle to the rbs

    return rbs