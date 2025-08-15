import math

pi = math.pi


class StanceController():
    """
    Stance phase controller. 

    Returns
    -------
    tau_h, tau_k, tau_a: hip, knee and ankle torques

    """

    qa_ref = 360 * pi / 180 * 10
    k_a = 2.7

    qk_ref = 226 * pi / 180 * 7
    k_k = 0.88

    q_lean_ref = 5 * pi / 180 * 30
    k_lean = 3.0
    k_lean_vel = 0.96

    r0 = 0.05
    qa0 = 90 * pi / 180
    qk0 = 120 * pi / 180
    k_gas = 3000

    def __init__(self, trunk_lean, qa, qk, qh):
        self.q_lean = trunk_lean

        self.qa = qa
        self.qk = qk
        self.qh = qh

        self.control_time_step = 0.001
        self.next_update_time = 0.0
        self.last_time = 0.0

        self.tau_a = 0.0
        self.tau_k = 0.0
        self.tau_h = 0.0

    def update(self, t, dt, q_lean, qa, qk, qh):

        if t >= self.next_update_time:

            # fix control time step if too small
            if self.control_time_step < dt:
                self.control_time_step = dt

            # compute lean velocity
            dq_lean = (q_lean - self.q_lean) / self.control_time_step

            dqa = (qa - self.qa) / self.control_time_step
            dqk = (qk - self.qk) / self.control_time_step

            # compute torques
            tau_a = self.k_a * (self.qa_ref - qa) - 10 * dqa
            tau_k = self.k_k * (self.qk_ref - qk) - 10 * dqk
            tau_h = self.k_lean * (self.q_lean_ref - q_lean) \
                + self.k_lean_vel * (0 - dq_lean)

            # add gastroc spring
            f_gas = self.k_gas * (self.r0 * (qk - self.qk0) - self.r0 * (qa - self.qa0))

            f_gas = f_gas * (f_gas > 0)

            #tau_a += f_gas * self.r0
            #tau_k += - f_gas * self.r0

            # print('f_gas: %4.1f, q_lean: %4.1f' % (f_gas, q_lean * 180 / pi))

            # update trunk lean angle
            self.q_lean = q_lean
            self.qa = qa
            self.qk = qk
            self.qh = qh

            self.dtau_a = (tau_a - self.tau_a) / self.control_time_step
            self.dtau_k = (tau_k - self.tau_k) / self.control_time_step
            self.dtau_h = (tau_h - self.tau_h) / self.control_time_step

            self.tau_a = tau_a
            self.tau_k = tau_k
            self.tau_h = tau_h

            self.last_time = t
            self.next_update_time = t + self.control_time_step

        else:

            # propagate linear torque change based on last velocity
            delta_t = t - self.last_time
            tau_h = self.tau_h + delta_t * self.dtau_h
            tau_k = self.tau_k + delta_t * self.dtau_k
            tau_a = self.tau_a + delta_t * self.dtau_a

        return tau_h, tau_k, tau_a
