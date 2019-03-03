import numpy as np

class PIDController:

    def __init__(self, k_p, k_d, k_i, dt=0):
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        self.integrated_error = 0.0
        self.dt = dt

    def step(self,
             x_target,
             x_actual,
             x_dot_target,
             x_dot_actual,
             x_dot_dot_ff
             ):
        # position error
        err = x_target - x_actual
        # integrated error
        self.integrated_error += err * self.dt
        # velocity error
        err_dot = x_dot_target - x_dot_actual
        # desired acceleration
        u_bar = self.k_p * err + self.k_d * err_dot + self.k_i * self.integrated_error + x_dot_dot_ff

        return u_bar


class LinearCascadingController:

    def __init__(self,
                 m,  # needed to convert u1_bar to u1
                 I_x,  # needed to convert u2_bar to u2
                 z_k_p=1.0,
                 z_k_d=1.0,
                 y_k_p=1.0,
                 y_k_d=1.0,
                 phi_k_p=1.0,
                 phi_k_d=1.0):
        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.phi_k_p = phi_k_p
        self.phi_k_d = phi_k_d

        self.g = 9.81
        self.I_x = I_x
        self.m = m
        self.pid_z = PIDController(self.z_k_p, self.z_k_d, 0.0)
        self.pid_y = PIDController(self.y_k_p, self.y_k_d, 0.0)
        self.pid_phi = PIDController(self.phi_k_p, self.phi_k_d, 0.0)

    def altitude_controller(self,
                            z_target,
                            z_actual,
                            z_dot_target,
                            z_dot_actual,
                            z_dot_dot_target,
                            phi_actual,  # unused parameter. Ignore for now.
                            ):
        """
        A PD controller which commands a thrust (u_1)
        for the vehicle.
        """

        # TODO (recommended to do AFTER attitude)
        #   Implement feedforward PD control to calculate
        #   u_1_bar and then use the linear math from above
        #   to transform u_1_bar into u_1 and then return u_1
        u1_bar = self.pid_z.step(z_target, z_actual, z_dot_target, z_dot_actual, z_dot_dot_target)
        u_1 = self.m * (self.g - u1_bar)
        return u_1

    def lateral_controller(self,
                           y_target,
                           y_actual,
                           y_dot_target,
                           y_dot_actual,
                           u_1=None,  # unused parameter. Ignore for now.
                           y_dot_dot_ff=0.0,
                           ):
        """
        A PD controller which commands a target roll
        angle (phi_commanded).
        """

        # TODO (recommended to do AFTER attitude)
        #   Implement feedforward PD control to calculate
        #   y_dot_dot_target and then use the linear math from above
        #   to transform y_dot_dot_target into phi_commanded
        #   and then return phi_commanded
        y_dot_dot_target = self.pid_y.step(y_target, y_actual, y_dot_target, y_dot_actual, y_dot_dot_ff)
        phi_commanded = y_dot_dot_target / self.g
        return phi_commanded

    def attitude_controller(self,
                            phi_target,
                            phi_actual,
                            phi_dot_actual,
                            phi_dot_target=0.0
                            ):
        """
        A PD controller which commands a moment (u_2)
        about the x axis for the vehicle.
        """

        # TODO (recommended to do FIRST)
        #   Implement PD control to calculate u_2_bar
        #   and then use the linear math from above to
        #   transform u_2_bar into u_2 and then return u_2
        u2_bar = self.pid_phi.step(phi_target, phi_actual, phi_dot_target, phi_dot_actual, 0.0)

        u_2 = self.I_x * u2_bar
        return u_2


class NonLinearCascadingController:

    def __init__(self,
                 m,
                 I_x,
                 z_k_p=1.0,
                 z_k_d=1.0,
                 y_k_p=1.0,
                 y_k_d=1.0,
                 phi_k_p=1.0,
                 phi_k_d=1.0):
        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.phi_k_p = phi_k_p
        self.phi_k_d = phi_k_d

        self.g = 9.81
        self.I_x = I_x
        self.m = m

    def altitude_controller(self,
                            z_target,
                            z_actual,
                            z_dot_target,
                            z_dot_actual,
                            z_dot_dot_target,
                            phi_actual):
        # TODO (recommended to do AFTER attitude)
        #   Implement feedforward PD control to calculate
        #   u_1_bar and then use the non-linear math from above
        #   to transform u_1_bar into u_1 and then return u_1

        # position error
        err = z_target - z_actual
        # velocity error
        err_dot = z_dot_target - z_dot_actual
        # desired acceleration
        u_1_bar = self.z_k_p * err + self.z_k_d * err_dot + z_dot_dot_target

        # nonlinear u_1
        u_1 = self.m * (self.g - u_1_bar) / np.cos(phi_actual)
        return u_1

    def lateral_controller(self,
                           y_target,
                           y_actual,
                           y_dot_target,
                           y_dot_actual,
                           u_1,
                           y_dot_dot_ff=0.0,
                           ):
        # TODO (recommended to do AFTER attitude)
        #   Implement feedforward PD control to calculate
        #   y_dot_dot_target and then use the non-linear math from above
        #   to transform y_dot_dot_target into phi_commanded
        #   and then return phi_commanded

        # position error
        err = y_target - y_actual
        # velocity error
        err_dot = y_dot_target - y_dot_actual
        # desired acceleration
        y_dot_dot_target = self.y_k_p * err + self.y_k_d * err_dot + y_dot_dot_ff

        # nonlinear u_2
        phi_commanded = np.arcsin((self.m * y_dot_dot_target) / u_1)

        return phi_commanded

    def attitude_controller(self,
                            phi_target,
                            phi_actual,
                            phi_dot_actual,
                            phi_dot_target=0.0
                            ):
        # TODO (recommended to do FIRST)
        #   Implement PD control to calculate u_2_bar
        #   and then use the linear math from above to
        #   transform u_2_bar into u_2 and then return u_2
        # position error
        err = phi_target - phi_actual
        # velocity error
        err_dot = phi_dot_target - phi_dot_actual
        # desired acceleration
        u_2_bar = self.phi_k_p * err + self.phi_k_d * err_dot

        u_2 = self.I_x * u_2_bar
        return u_2

