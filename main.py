import time
from config import human_model
# from control import StanceController
from output import RbsAnimation, plot_joint_angles, plot_joint_torques, plot_grf, plot_xzp, plot_activations, plot_F_mtc, plot_f_l, plot_f_v
from trackers import ContactTracker
import matplotlib.pyplot as plt

# INTEGRATION PARAMETERS
dt = 1e-5                  # [s] integration time step
tStop = 2            # [s] simulation stop time

print('\033[H\033[J')  # clear screen (equivalent to Matlab 'clc')
# Close all figures
plt.close('all')

# create rigid body system
rbs = human_model(dt)

# bodies stored in variables
trunk = rbs.body_list[0]
thigh = rbs.body_list[1]
shank = rbs.body_list[2]
# foot = rbs.body_list[3]

ball = rbs.contact_list[0]
ball_tracker = ContactTracker(ball)

# joints stored in variables
hip = rbs.joint_list[0]
knee = rbs.joint_list[1]
# ankle = rbs.joint_list[2]

jump_muscle = rbs.muscle_list[0]

# stance_ctrl = StanceController(trunk.p, ankle.q, knee.q, hip.q)

qh = [hip.q] # joint angles list, used for plotting
tauh = [hip.tau] # joint torques list, used for plotting

rbs_anim = RbsAnimation(update_time_step=dt, rigid_body_system=rbs, adaptive= True, ratio=2)

# print('moving on')

start_time = time.time()

t = [0.0]

# integration while loop
while t[-1] <= tStop:

    # animate rbs
    # print('going to updATE THE ANIMATION')
    rbs_anim.update_animation(t[-1], rigid_body_system=rbs)

    # append joint angles to a list for plotting
    # qh.append(hip.q)
    # qk.append(knee.q)
    # qa.append(ankle.q)

    # # muscle produces force if the foot is in stance
    # if ball.contact is True:
    #     # tau_h, tau_k, tau_a = stance_ctrl.update(t[-1], dt, trunk.p, ankle.q, knee.q, hip.q)
    #     tau_h, tau_k = 0.0, 0.0
    # else:
    #     tau_h, tau_k = 0.0, 0.0

    # print('tau_h: %4.2f, tau_k: %4.2f, tau_a: %4.2f\n' % (tau_h, tau_k, tau_a))

    # Store joint torques in a list for plotting
    # tauh.append(hip.tau)
    # tauk.append(knee.tau)
    # taua.append(ankle.tau)

    # update contact point
    ball.update(dt, ground_height=0)
    ball_tracker.append()

    # print(tau_h)

    # Need to update the mtc as well as the activation here!!!!!
    
    # update forces and torques acting at all joints
    hip.update(dt,0)
    knee.update(dt,0)
    # ankle.update(dt, tau_a)

    if ball.contact is True:
        jump_muscle.update(dt)
    else:
        jump_muscle.reset()
    
    # updating activation list
    jump_muscle.A_list.append(jump_muscle.A)
    jump_muscle.F_mtc_list.append(jump_muscle.F_mtc)
    jump_muscle.f_l_list.append(jump_muscle.f_l)
    jump_muscle.f_v_list.append(jump_muscle.f_v)
    
    # if t[-1] >= 0.456:
        # check = jump_muscle.f_l_list[-1]

    # integrate a single timestep
    for body in rbs.body_list:
        body.integrate(dt)

        # print('integrated')

    t.append(t[-1] + dt)
    # print(len(t))

print('TOTAL time: %f sec.' % (time.time() - start_time))

plot_activations(t , jump_muscle.A_list)

plot_F_mtc(t , jump_muscle.F_mtc_list)

plot_f_l(t , jump_muscle.f_l_list)

plot_f_v(t , jump_muscle.f_v_list)

# plot_joint_angles(t, qk, qk, qh)
# plot_joint_torques(t, tauk, tauk, tauh)

plot_xzp(trunk, t, 22, "trunk position")  

ball_tracker.plot(t, 89, 'Ball contact point of foot segment') # fig_num- just the heading on the figure window (eg: figure 1)