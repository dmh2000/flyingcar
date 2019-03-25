import numpy as np
import math
from math import sin, cos, tan, sqrt
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from mpl_toolkits.mplot3d import Axes3D
# import jdc
import random

from solution import UDACITYDroneIn3D, UDACITYController
import testing
import sys

pylab.rcParams['figure.figsize'] = 10, 10


class DroneIn3D(UDACITYDroneIn3D):

    def __init__(self,
                 k_f=1.0,
                 k_m=1.0,
                 m=0.5,
                 L=0.566,  # full rotor to rotor distance
                 i_x=0.1,
                 i_y=0.1,
                 i_z=0.2):
        self.k_f = k_f
        self.k_m = k_m
        self.m = m
        self.l = L / (2 * sqrt(2))  # perpendicular distance to axes
        self.i_x = i_x
        self.i_y = i_y
        self.i_z = i_z

        # x, y, y, phi, theta, psi,
        # x_dot, y_dot, z_dot, p, q, r
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.omega = np.array([0.0, 0.0, 0.0, 0.0])

        self.g = 9.81

    @property
    def I(self):
        return np.array([self.i_x, self.i_y, self.i_z])

    # euler angles [rad] (in world / lab frame)
    @property
    def phi(self):
        return self.X[3]

    @property
    def theta(self):
        return self.X[4]

    @property
    def psi(self):
        return self.X[5]

    @property
    def euler_angles(self):
        return self.X[3:6]

    # body rates [rad / s] (in body frame)
    @property
    def p(self):
        return self.X[9]

    @property
    def q(self):
        return self.X[10]

    @property
    def r(self):
        return self.X[11]

    @property
    def pqr(self):
        return self.X[9:12]

    # propeller linear forces are proportional to rotation rate squared
    # Kf is a constant derived empiraclly

    # forces from the four propellers [N]
    @property
    def f_1(self):
        f = self.k_f * self.omega[0] ** 2
        return f

    @property
    def f_2(self):
        f = self.k_f * self.omega[1] ** 2
        return f

    @property
    def f_3(self):
        f = self.k_f * self.omega[2] ** 2
        return f

    @property
    def f_4(self):
        f = self.k_f * self.omega[3] ** 2
        return f

    # collective force
    @property
    def f_total(self):
        f_t = self.f_1 + self.f_2 + self.f_3 + self.f_4
        return f_t

    # reactive moments are also proporational to rotation rate squared, with Km instead of Kf
    # reactive moments [N * m]
    @property
    def tau_1(self):
        tau = -self.k_m * self.omega[0] ** 2
        return tau

    @property
    def tau_2(self):
        tau = self.k_m * self.omega[1] ** 2
        return tau

    @property
    def tau_3(self):
        tau = -self.k_m * self.omega[2] ** 2
        return tau

    @property
    def tau_4(self):
        tau = self.k_m * self.omega[3] ** 2
        return tau

    # moments about axes [N * m]
    @property
    def tau_x(self):
        tau = self.l * (self.f_1 + self.f_4 - self.f_2 - self.f_3)
        return tau

    @property
    def tau_y(self):
        tau = self.l * (self.f_1 + self.f_2 - self.f_3 - self.f_4)
        return tau

    @property
    def tau_z(self):
        tau = self.tau_1 + self.tau_2 + self.tau_3 + self.tau_4
        return tau

    @property
    def tau_xyz(self):
        return [self.tau_x, self.tau_y, self.tau_z]

    def set_propeller_angular_velocities(self,
                                         c,
                                         u_bar_p,
                                         u_bar_q,
                                         u_bar_r):
        # TODO replace with your own implementation.
        #   note that this function doesn't return anything
        #   it just sets self.omega
        #
        c_bar = (-c * self.m) / self.k_f
        p_bar = (self.i_x * u_bar_p) / (self.k_f * self.l)
        q_bar = (self.i_y * u_bar_q) / (self.k_f * self.l)
        r_bar = (self.i_z * u_bar_r) / self.k_m
        # isolate omega[0] by adding all four equations
        # c_bar = O[0] + O[1] + O[2] + O[3]
        # p_bar = O[0] - O[1] - O[2] + O[3]
        # q_bar = O[0] + O[1] - O[2] - O[3]
        # r_bar = O[0] - O[1] + O[2] - O[3]

        # -----------------------------------
        #        4*O[0] + 0*O[1] + 0*O[2] + 0*O[3]
        # c_bar + p_bar + q_bar + r_bar = 4 * omega[0]
        # cbar = 4*O[0]
        o0 = (c_bar + p_bar + q_bar + r_bar) / 4
        if o0 < 0.0:
            o0 = 0.0
        else:
            o0 = np.sqrt(o0)
        # isolate omega[1] by adding c_bar and q_bar
        # c_bar = O[0] + O[1] + O[2] + O[3]
        # q_bar = O[0] + O[1] - O[2] - O[3]
        # -----------------------------------
        # c_bar + qbar = 2*O[0] + 2*O[1]
        # omega[0] (o0) is already known so plug in the value
        o1 = (c_bar + q_bar - 2 * o0) / 2.0
        if o1 < 0.0:
            o1 = 0.0
        else:
            o1 = np.sqrt(o1)
        # isolate omega[2] by adding c_bar and r_bar
        # c_bar = O[0] + O[1] + O[2] + O[3]
        # r_bar = O[0] - O[1] + O[2] - O[3]
        # -----------------------------------
        # c_bar + r_bar = 2 * O[0] + 0 + 2 * O[2] + 0
        o2 = (c_bar + r_bar - 2 * o0) / 2.0
        if o2 < 0.0:
            o2 = 0.0
        else:
            o2 = np.sqrt(o2)
        # isolate omega[3] by adding c_bar and p_bar
        # c_bar = O[0] + O[1] + O[2] + O[3]
        # p_bar = O[0] - O[1] - O[2] + O[3]
        # -----------------------------------
        # c_bar + p_bar = 2 * O[0] + 0 + 2 * O[3] + 0
        o3 = (c_bar + p_bar - 2 * o0) / 2.0
        if o3 < 0.0:
            o3 = 0.0
        else:
            o3 = np.sqrt(o3)

        # super(DroneIn3D, self).set_propeller_angular_velocities(c,
        #                                                               u_bar_p,
        #                                                               u_bar_q,
        #                                                               u_bar_r)
        self.omega[0] = o0  # clockwise
        self.omega[1] = o1  # counter clockwise
        self.omega[2] = o2  # clockwise
        self.omega[3] = o3  # counter clockwise

    # Exercise 1.2
    def R_zyx(self, phi, theta, psi):
        # TODO replace with your own implementation
        #   according to the math above
        #
        # return rotation_matrix

        # precompute cos and sin
        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)
        # create rx matrix
        rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cos_phi, -sin_phi],
            [0.0, sin_phi, cos_phi]
        ])

        # precompute cos and sin
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        # create ry matrix
        ry = np.array([
            [cos_theta, 0.0, sin_theta],
            [0.0, 1.0, 0.0],
            [-sin_theta, 0, cos_theta]
        ])

        # precompute cos and sin
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        # create rz matrix
        rz = np.array([
            [cos_psi, -sin_psi, 0.0],
            [sin_psi, cos_psi, 0.0],
            [0.0, 0.0, 1.0]
        ])

        # multiply rz * ry * rx
        r = np.matmul(rz, np.matmul(ry, rx))
        return r

    def R(self):
        r1 = self.R_zyx(self.phi, self.theta, self.psi)

        # check against solution
        # r0 = super(DroneIn3D, self).R()
        # if not np.allclose(r0,r1):
        #    print(r0,r1)

        return r1

    def linear_acceleration(self):

        # TODO replace with your own implementation
        #   This function should return a length 3 np.array
        #   with a_x, a_y, and a_z
        # compute Ftotal
        f_total = 0;
        f_total += self.k_f * self.omega[0] ** 2
        f_total += self.k_f * self.omega[1] ** 2
        f_total += self.k_f * self.omega[2] ** 2
        f_total += self.k_f * self.omega[3] ** 2
        # rotate F by the current rotation matrix
        r = self.R().dot([0.0, 0.0, -self.f_total])
        # divide by mass
        r = r / self.m
        # add in gravity
        r = r + [0.0, 0.0, self.g]

        # compare to solution
        # la = super(DroneIn3D,self).linear_acceleration()
        # if not np.allclose(r,la):
        #    print(r,la)
        return r

    def get_omega_dot(self):
        # TODO replace with your own implementation
        # vector form with properties added to return tau_xyz,self.pqr,self.I
        o_dot = (self.tau_xyz - np.cross(self.pqr, self.I * self.pqr)) / self.I

        # check against solution
        # s_dot = super(DroneIn3D, self).get_omega_dot()
        # if not np.allclose(s_dot,o_dot):
        #    print(s_dot,o_dot)
        return o_dot

    def get_euler_derivatives(self):
        # TODO - replace with your own implementation
        #   return np.array([phi_dot, theta_dot, psi_dot])
        sin_phi = np.sin(self.phi)
        cos_phi = np.cos(self.phi)
        tan_tht = np.tan(self.theta)
        cos_tht = np.cos(self.theta)
        e = np.array([
            [1.0, sin_phi * tan_tht, cos_phi * tan_tht],
            [0.0, cos_phi, -1.0 * sin_phi],
            [0.0, sin_phi / cos_tht, cos_phi / cos_tht]
        ])
        e_dot = np.dot(e, self.pqr)

        # check against solution
        # s_dot = super(DroneIn3D, self).get_euler_derivatives()
        # if not np.allclose(e_dot,s_dot):
        #     print(e_dot,s_dot)

        return e_dot

    def advance_state(self, dt):

        # TODO replace this with your own implementation
        #
        #   make sure this function returns the new state!
        #   Some of the code that calls this function will expect
        #   it to return the state, so simply updating self.X
        #   is not enough (though you should do that in this
        #   method too.)
        xd = self.X[6:9]
        ed = self.get_euler_derivatives()
        la = self.linear_acceleration()
        od = self.get_omega_dot()
        # create the vector of velocities and accelerations
        x_dot = np.concatenate((xd, ed, la, od))
        # x_dot = np.array([
        #     xd[0],    # x_dot
        #     xd[1],    # y_dot
        #     xd[2],    # z_dot
        #     ed[0],    # phi_dot
        #     ed[1],    # theta_dot
        #     ed[2],    # psi_dot
        #     la[0],    # x_dot_dot
        #     la[1],    # y_dot_dot
        #     la[2],    # z_dot_dot
        #     od[0],    # p_dot
        #     od[1],    # q_dot
        #     od[2]     # r_dot
        #  ])


        #
        x = self.X + x_dot * dt

        # check against solution
        # s_x = super(DroneIn3D, self).advance_state(dt)
        # if not np.allclose(s_x,x):
        #     print(s_x,x)

        self.X = x
        return self.X


class Controller(UDACITYController):

    def __init__(self,
                 z_k_p=1.0,
                 z_k_d=1.0,
                 x_k_p=1.0,
                 x_k_d=1.0,
                 y_k_p=1.0,
                 y_k_d=1.0,
                 k_p_roll=1.0,
                 k_p_pitch=1.0,
                 k_p_yaw=1.0,
                 k_p_p=1.0,
                 k_p_q=1.0,
                 k_p_r=1.0):
        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.x_k_p = x_k_p
        self.x_k_d = x_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.k_p_roll = k_p_roll
        self.k_p_pitch = k_p_pitch
        self.k_p_yaw = k_p_yaw
        self.k_p_p = k_p_p
        self.k_p_q = k_p_q
        self.k_p_r = k_p_r

        print('x: delta = %5.3f' % (x_k_d / 2 / math.sqrt(x_k_p)), ' omega_n = %5.3f' % (math.sqrt(x_k_p)))
        print('y: delta = %5.3f' % (y_k_d / 2 / math.sqrt(y_k_p)), ' omega_n = %5.3f' % (math.sqrt(y_k_p)))
        print('z: delta = %5.3f' % (z_k_d / 2 / math.sqrt(z_k_p)), ' omega_n = %5.3f' % (math.sqrt(z_k_p)))

        self.g = 9.81

    # Exercise 4.1

    def lateral_controller(self,
                           x_target,
                           x_dot_target,
                           x_dot_dot_target,
                           x_actual,
                           x_dot_actual,
                           y_target,
                           y_dot_target,
                           y_dot_dot_target,
                           y_actual,
                           y_dot_actual,
                           c):
        # TODO replace with your own implementation
        # return b_x_c, b_y_c

        # x controller
        x_err = x_target - x_actual
        x_err_dot = x_dot_target - x_dot_actual
        x_cmd = self.x_k_p * x_err + self.x_k_d * x_err_dot + x_dot_dot_target
        b_x_c = x_cmd / c

        # y controller
        y_err = y_target - y_actual
        y_err_dot = y_dot_target - y_dot_actual
        y_cmd = self.y_k_p * y_err + self.y_k_d * y_err_dot + y_dot_dot_target
        b_y_c = y_cmd / c
        return b_x_c, b_y_c

        # return super(Controller, self).lateral_controller(
        #    x_target,
        #    x_dot_target,
        #    x_dot_dot_target,
        #    x_actual,
        #    x_dot_actual,
        #    y_target,
        #    y_dot_target,
        #    y_dot_dot_target,
        #    y_actual,
        #    y_dot_actual,
        #    c)

    # Exercise 4.2

    def roll_pitch_controller(self,
                              b_x_c_target,
                              b_y_c_target,
                              rot_mat):
        # TODO replace with your own implementation
        # return p_c, q_c

        # extract terms from rotation matrix
        r11 = rot_mat[0, 0]
        r12 = rot_mat[0, 1]
        r13 = rot_mat[0, 2]

        r21 = rot_mat[1, 0]
        r22 = rot_mat[1, 1]
        r23 = rot_mat[1, 2]

        r33 = rot_mat[2, 2]

        # commanded b_x_dot
        b_x_c_actual = r13
        b_x_c_err = b_x_c_target - b_x_c_actual
        b_x_c_dot = self.k_p_roll * b_x_c_err

        # commanded b_y_dot
        b_y_c_actual = r23
        b_y_c_err = b_y_c_target - b_y_c_actual
        b_y_c_dot = self.k_p_pitch * b_y_c_err

        # transform to p,q
        b = np.array([b_x_c_dot, b_y_c_dot])
        r = np.array([
            [r21, -r11],
            [r22, -r12]]
        ) / r33
        pq = np.matmul(r, b)
        p_c = pq[0]
        q_c = pq[1]

        # print(p,q)
        # p,q = super(Controller, self).roll_pitch_controller(b_x_c_target,
        #                                                    b_y_c_target,
        #                                                    rot_mat)
        # print(p,q)
        return p_c, q_c

    # Exercise 5.1

    def body_rate_controller(self,
                             p_c,
                             q_c,
                             r_c,
                             p_actual,
                             q_actual,
                             r_actual):
        # TODO replace with your own implementation
        # return u_bar_p, u_bar_q, u_bar_r

        p_err = p_c - p_actual
        u_bar_p = self.k_p_p * p_err

        q_err = q_c - q_actual
        u_bar_q = self.k_p_q * q_err

        r_err = r_c - r_actual
        u_bar_r = self.k_p_r * r_err

        return u_bar_p, u_bar_q, u_bar_r
        # return super(Controller, self).body_rate_controller(p_c,
        #                                                     q_c,
        #                                                     r_c,
        #                                                     p_actual,
        #                                                     q_actual,
        #                                                     r_actual)

    # Exercise 5.2

    def yaw_controller(self,
                       psi_target,
                       psi_actual):
        # TODO replace with your own implementation
        # return r_c
        psi_err = psi_target - psi_actual
        r_c = self.k_p_yaw * psi_err
        return r_c

        # return super(Controller, self).yaw_controller(psi_target,
        #                                              psi_actual)

    # Exercise 5.3

    def altitude_controller(self,
                            z_target,
                            z_dot_target,
                            z_dot_dot_target,
                            z_actual,
                            z_dot_actual,
                            rot_mat):
        # TODO replace with your own implementation
        # return c
        z_err = z_target - z_actual
        z_dot_err = z_dot_target - z_dot_actual
        u_1_bar = self.z_k_p * z_err + self.z_k_d * z_dot_err + z_dot_dot_target
        c = (u_1_bar - self.g) / rot_mat[2, 2]
        return c  # u_1

        # return super(Controller, self).altitude_controller(z_target,
        #                                                   z_dot_target,
        #                                                   z_dot_dot_target,
        #                                                   z_actual,
        #                                                   z_dot_actual,
        #                                                   rot_mat)

    def attitude_controller(self,
                            b_x_c_target,
                            b_y_c_target,
                            psi_target,
                            psi_actual,
                            p_actual,
                            q_actual,
                            r_actual,
                            rot_mat):
        p_c, q_c = self.roll_pitch_controller(b_x_c_target,
                                              b_y_c_target,
                                              rot_mat)

        r_c = self.yaw_controller(psi_target,
                                  psi_actual)

        u_bar_p, u_bar_q, u_bar_r = self.body_rate_controller(
            p_c,
            q_c,
            r_c,
            p_actual,
            q_actual,
            r_actual)

        return u_bar_p, u_bar_q, u_bar_r


total_time = 20.0
dt = 0.01

t = np.linspace(0.0, total_time, int(total_time / dt))

omega_x = 0.8
omega_y = 0.4
omega_z = 0.4

a_x = 1.0
a_y = 1.0
a_z = 1.0

x_path = a_x * np.sin(omega_x * t)
x_dot_path = a_x * omega_x * np.cos(omega_x * t)
x_dot_dot_path = -a_x * omega_x ** 2 * np.sin(omega_x * t)

y_path = a_y * np.cos(omega_y * t)
y_dot_path = -a_y * omega_y * np.sin(omega_y * t)
y_dot_dot_path = -a_y * omega_y ** 2 * np.cos(omega_y * t)

z_path = a_z * np.cos(omega_z * t)
z_dot_path = -a_z * omega_z * np.sin(omega_z * t)
z_dot_dot_path = - a_z * omega_z ** 2 * np.cos(omega_z * t)

psi_path = np.arctan2(y_dot_path, x_dot_path)

# plotting drone headings
fig = plt.figure()
ax = fig.gca(projection='3d')

u = np.cos(psi_path)
v = np.sin(psi_path)
w = np.zeros(psi_path.shape)
for i in range(0, z_path.shape[0], 20):
    ax.quiver(x_path[i], y_path[i], z_path[i], u[i], v[i], w[i], length=0.2, normalize=True, color='green')

plt.title('Flight path').set_fontsize(20)
ax.set_xlabel('$x$ [$m$]').set_fontsize(20)
ax.set_ylabel('$y$ [$m$]').set_fontsize(20)
ax.set_zlabel('$z$ [$m$]').set_fontsize(20)
plt.legend(['Planned yaw', ], fontsize=14)

plt.show()

# EXECUTING THE FLIGHT
# how fast the inner loop (Attitude controller) performs calculations
# relative to the outer loops (altitude and position controllers).
inner_loop_relative_to_outer_loop = 10

# creating the drone object
drone = DroneIn3D()


# creating the control system object
def time_constant(t, damping):
    kp = (1.0 / t ** 2) * (1.0 + 2.0 * damping)
    kd = (1.0 / t) * (1.0 + 2.0 * damping)
    ki = 1.0 / t ** 3
    print(kp, kd, ki)
    return kp, kd, ki


"""
control_system = Controller(z_k_p=2.0, 
                            z_k_d=1.0, 
                            x_k_p=6.0,
                            x_k_d=4.0,
                            y_k_p=6.0,
                            y_k_d=4.0,
                            k_p_roll=8.0,
                            k_p_pitch=8.0,
                            k_p_yaw=8.0,
                            k_p_p=20.0,
                            k_p_q=20.0,
                            k_p_r=k_p_r)
"""

z_k_p = 15.0
z_k_d = 1.0
x_k_p = 6.0
x_k_d = 4.0
y_k_p = 6.0
y_k_d = 4.0
k_p_roll = 8.0
k_p_pitch = 8.0
k_p_yaw = 1.0
k_p_p = 20.0
k_p_q = 20.0
k_p_r = -10.0 # 0.003

# control_system = UDACITYController(
control_system = Controller(
    z_k_p=z_k_p,
    z_k_d=z_k_d,
    x_k_p=x_k_p,
    x_k_d=x_k_d,
    y_k_p=y_k_p,
    y_k_d=y_k_d,
    k_p_roll=k_p_roll,
    k_p_pitch=k_p_pitch,
    k_p_yaw=k_p_yaw,
    k_p_p=k_p_p,
    k_p_q=k_p_q,
    k_p_r=k_p_r)

# declaring the initial state of the drone with zero
# height and zero velocity

drone.X = np.array([x_path[0],  # x
                    y_path[0],  # y
                    z_path[0],  # z
                    0.0,  # roll
                    0.0,  # pitch
                    psi_path[0],  # yaw
                    x_dot_path[0],  # x_dot
                    y_dot_path[0],  # y_dot
                    z_dot_path[0],  # z_dot
                    0.0,  # p
                    0.0,  # q
                    0.0])  # r

# arrays for recording the state history,
# propeller angular velocities and linear accelerations
drone_state_history = drone.X
omega_history = drone.omega
accelerations = drone.linear_acceleration()
accelerations_history = accelerations
angular_vel_history = drone.get_euler_derivatives()

# executing the flight
for i in range(0, z_path.shape[0]):

    rot_mat = drone.R()

    c = control_system.altitude_controller(z_path[i],
                                           z_dot_path[i],
                                           z_dot_dot_path[i],
                                           drone.X[2],
                                           drone.X[8],
                                           rot_mat)

    b_x_c, b_y_c = control_system.lateral_controller(x_path[i],
                                                     x_dot_path[i],
                                                     x_dot_dot_path[i],
                                                     drone.X[0],
                                                     drone.X[6],
                                                     y_path[i],
                                                     y_dot_path[i],
                                                     y_dot_dot_path[i],
                                                     drone.X[1],
                                                     drone.X[7],
                                                     c)

    for _ in range(inner_loop_relative_to_outer_loop):
        rot_mat = drone.R()

        angular_vel = drone.get_euler_derivatives()

        u_bar_p, u_bar_q, u_bar_r = control_system.attitude_controller(
            b_x_c,
            b_y_c,
            psi_path[i],
            drone.psi,
            drone.X[9],
            drone.X[10],
            drone.X[11],
            rot_mat)

        drone.set_propeller_angular_velocities(c, u_bar_p, u_bar_q, u_bar_r)

        drone_state = drone.advance_state(dt / inner_loop_relative_to_outer_loop)
        # print(psi_path[i],drone.psi)

    # generating a history of the state history, propeller angular velocities and linear accelerations
    drone_state_history = np.vstack((drone_state_history, drone_state))

    omega_history = np.vstack((omega_history, drone.omega))
    accelerations = drone.linear_acceleration()
    accelerations_history = np.vstack((accelerations_history, accelerations))
    angular_vel_history = np.vstack((angular_vel_history, drone.get_euler_derivatives()))

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x_path, y_path, z_path, linestyle='-', marker='.', color='red')
ax.plot(drone_state_history[:, 0],
        drone_state_history[:, 1],
        drone_state_history[:, 2],
        linestyle='-', color='blue')

plt.title('Flight path').set_fontsize(20)
ax.set_xlabel('$x$ [$m$]').set_fontsize(20)
ax.set_ylabel('$y$ [$m$]').set_fontsize(20)
ax.set_zlabel('$z$ [$m$]').set_fontsize(20)
plt.legend(['Planned path', 'Executed path'], fontsize=14)

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)

plt.show()

# FLIGHT PATH COMPARISON
fig = plt.figure()
ax = fig.gca(projection='3d')

u = np.cos(psi_path)
v = np.sin(psi_path)
w = np.zeros(psi_path.shape)

a = psi_path
b = drone_state_history[:, 5]

drone_u = np.cos(drone_state_history[:, 5])
drone_v = np.sin(drone_state_history[:, 5])
drone_w = np.zeros(psi_path.shape)

for i in range(0, z_path.shape[0], 20):
    ax.quiver(x_path[i], y_path[i], z_path[i], u[i], v[i], w[i], length=0.2, normalize=True, color='red')
    ax.quiver(drone_state_history[i, 0],
              drone_state_history[i, 1],
              drone_state_history[i, 2],
              drone_u[i], drone_v[i], drone_w[i],
              length=0.2, normalize=True, color='blue')
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
plt.title('Flight path').set_fontsize(20)
ax.set_xlabel('$x$ [$m$]').set_fontsize(20)
ax.set_ylabel('$y$ [$m$]').set_fontsize(20)
ax.set_zlabel('$z$ [$m$]').set_fontsize(20)
plt.legend(['Planned yaw', 'Executed yaw'], fontsize=14)

plt.show()

# ERROR IN FLIGHT POSITION
err = np.sqrt((x_path - drone_state_history[:-1, 0]) ** 2
              + (y_path - drone_state_history[:-1, 1]) ** 2
              + (y_path - drone_state_history[:-1, 2]) ** 2)

plt.plot(t, err)
plt.title('Error in flight position').set_fontsize(20)
plt.xlabel('$t$ [$s$]').set_fontsize(20)
plt.ylabel('$e$ [$m$]').set_fontsize(20)
plt.show()

# ANGULAR VELOCITIES
plt.plot(t, -omega_history[:-1, 0], color='blue')
plt.plot(t, omega_history[:-1, 1], color='red')
plt.plot(t, -omega_history[:-1, 2], color='green')
plt.plot(t, omega_history[:-1, 3], color='black')

plt.title('Angular velocities').set_fontsize(20)
plt.xlabel('$t$ [$s$]').set_fontsize(20)
plt.ylabel('$\omega$ [$rad/s$]').set_fontsize(20)
plt.legend(['P 1', 'P 2', 'P 3', 'P 4'], fontsize=14)

plt.grid()
plt.show()

# PLOTTING THE YAW ANGLE OF THE DRONE IN TIME
plt.plot(t, psi_path, marker='.')
plt.plot(t, drone_state_history[:-1, 5])
plt.title('Yaw angle').set_fontsize(20)
plt.xlabel('$t$ [$s$]').set_fontsize(20)
plt.ylabel('$\psi$ [$rad$]').set_fontsize(20)
plt.legend(['Planned yaw', 'Executed yaw'], fontsize=14)
plt.show()
