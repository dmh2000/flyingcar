#!/usr/bin/env python
# coding: utf-8

# ## Co-axial drone dynamics
# 
# <img src="Drone1.png" width="300">
# 
# In this exercise, you will populate the ```CoaxialCopter``` class with three methods. These functions will calculate vertical acceleration $\ddot{z}$, angular acceleration $\ddot{\psi}$ along the $z$ axis and the angular velocities $\omega_1$ and $\omega_2$ of the propellers required to achieve any desired  $\ddot{z}$ and $\ddot{\psi}$. 
# 
# We assume that drone only can travel and rotate along the vertical $z$ axis. 
# 
# Remember, the positive $z$ axis points downwards. 
# 
# Also notice that the first propeller rotates clockwise, while the second propeller rotates counterclockwise.
# 
# **Reminder**: The second propeller rotates counterclockwise thus the angular velocity needs to be positive, while angular velocity of the first propeller needs to be negative as it rotates clockwise.
# 

# In[1]:


get_ipython().run_line_magic('matplotlib', 'inline')
get_ipython().run_line_magic('config', "InlineBackend.figure_format = 'retina'")

import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import jdc
from ExerciseAnswers import Answers

pylab.rcParams['figure.figsize'] = 10, 10


# #### Helpful Equations
# 
# $$F_z = m \ddot{z}$$
# 
# $$M_z = I_z \ddot{\psi}$$
# 
# $$ \ddot{z} = g- \frac{k_f}{m}\left( \omega_1^2 +\omega_2^2 \right)   \\
#    \ddot{\psi} =\frac{k_m}{I_z} (-\omega_1^2 + \omega_2^2 )  $$
#    
# _Hint_:
# From those equations you can derive the following:
# 
# $$ \omega_1^2 = \frac{m}{k_f} (g-\ddot{z}) - \omega_2^2 $$
# 
# $$ \omega_2^2 = \frac{I_z}{k_m} \ddot{\psi} + \omega_1^2 $$
# 
# and replacing the omega_squared variables in the two equations leads to:
# 
# $$ \omega_1^2 =\frac{1}{2}\frac{m}{k_f} (g-\ddot{z}) - \frac{1}{2}\frac{I_z}{k_m} \ddot{\psi} $$
# 
# $$ \omega_2^2 =\frac{1}{2}\frac{m}{k_f} (g-\ddot{z}) + \frac{1}{2}\frac{I_z}{k_m} \ddot{\psi} $$
# 
# $$ $$
# $$ $$
# _Note_: In the notebook we will refer to 
# $ \ddot{\psi} $ as angular_acceleration and 
# $\ddot{z}$ as linear_acceleration

# In[18]:


class CoaxialCopter:
    
    def __init__(self, 
                 k_f = 0.1, # value of the thrust coefficient
                 k_m = 0.1, # value of the angular torque coefficient
                 m = 0.5,   # mass of the vehicle 
                 i_z = 0.2, # moment of inertia around the z-axis
                ):
        
        self.k_f = k_f
        self.k_m = k_m
        self.m = m
        self.i_z = i_z
        
        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = 9.81

        self.z = 0.0
        self.z_dot = 0.0
        self.psi = 0.0
        self.psi_dot = 0.0
        self.y = 0.0
        self.y_dot = 0.0
    
    @property
    def z_dot_dot(self):
        """Calculates current vertical acceleration."""
        
        # TODO: 
        # 1. Calculate the lift force generated by the first 
        #    and second propellers 
        # 2. Calculate the total vertical force acting on the drone 
        # 3. Calculate the vertical acceleration due to the 
        #    total force acting on the drone keep in mind that the 
        #    z-axis is directed downward 
        acceleration = self.g - (self.k_f / self.m) * (self.omega_1**2 + self.omega_2**2)
        return acceleration
    
    @property
    def psi_dot_dot(self): 
        """Calculates current rotational acceleration."""
        
        # TODO: 
        # 1. Calculate the torques generated by both propellers 
        # 2. Calculate the angular acceleration 
        # clocksize
        M1 = self.k_m * self.omega_1**2
        P1 = M1 / self.i_z
        # counter clocksize|
        M2 = self.k_m * self.omega_2**2
        P2 = -M2 / self.i_z
        
        angular_acc = P1 + P2
        return angular_acc
    
    def set_rotors_angular_velocities(self, linear_acc, angular_acc): 
        """
        Sets the turn rates for the rotors so that the drone
        achieves the desired linear_acc and angular_acc.
        """
        
        # TODO 
        # 1. Calculate the correct values of omega_1 and omega_2
        # 2. Set self.omega_1 and self.omega_2 to those values
        # 3. Don't forget to return omega_1, omega_2
        # Reminder: The second propeller rotates counterclockwise 
        # thus the angular velocity needs to be positive, 
        # while angular velocity of the first propeller needs to be 
        # negative as it rotates clockwise.
        self.omega_1 = -1.0 * np.sqrt(0.5 * (self.m / self.k_f) * (self.g - linear_acc) - 0.5 * (self.i_z / self.k_m * angular_acc))
        self.omega_2 = -self.omega_1
        return self.omega_1, self.omega_2

    def advance_state_uncontrolled(self,dt):
        # update z_dot
        z_dot_dot = self.g
        delta_z_dot = z_dot_dot * dt
        self.z_dot = self.z_dot + delta_z_dot

        # update y_dot
        y_dot_dot = 0.0
        delta_y_dot = y_dot_dot * dt
        self.y_dot = self.y_dot + delta_y_dot

    def advance_state(self,dt):
        #
        # TODO
        #  Implement this method! Your implementation may look
        #  VERY similar to the uncontrolled version of this function.
        # update z_dot
        z_dot_dot = self.g
        delta_z_dot = z_dot_dot * dt
        self.X[3] += delta_z_dot

        # update  y_dot
        y_dot_dot = 0.0
        delta_y_dot = y_dot_dot * dt
        self.X[4] += delta_y_dot

        delta_X = np.array([self.X[3], self.X[4], 0.0, 0.0, 0.0, 0.0]) * dt
        self.X += delta_X


# In[19]:


# TEST CODE 1

bi = CoaxialCopter()
stable_omega_1,stable_omega_2 = bi.set_rotors_angular_velocities(0.0, 0.0)

print('Drone achieves stable hover with angular velocity of %5.2f' % stable_omega_1, 
      'for the first propeller and %5.2f' % stable_omega_2, 
      'for the second propeller.')

Answers.angular_velocities(bi.m, bi.g, bi.i_z, bi.k_f, bi.k_m, 0.0, 0.0, stable_omega_1, stable_omega_2)


# In[20]:


# TEST CODE 2 - Checking the linear acceleration value

bi.omega_1 = stable_omega_1 * math.sqrt(1.1)
bi.omega_2 = stable_omega_2 * math.sqrt(1.1)

vertical_acceleration = bi.z_dot_dot
print('Increase by %5.2f' % math.sqrt(1.1),
      'of the propeller angular velocity will result in',
      '%5.2f' % vertical_acceleration, 
      'm/(s*s) vertical acceleration.' )

Answers.linear_acceleration(bi.m, bi.g, bi.k_f, bi.omega_1, bi.omega_2, vertical_acceleration)


# In[21]:


# TEST CODE 3 - checking the angular acceleration
bi.omega_1 = stable_omega_1 * math.sqrt(1.1)
bi.omega_2 = stable_omega_2 * math.sqrt(0.9)

ang_acceleration = bi.psi_dot_dot
print('Increase in %5.2f'%math.sqrt(1.1),' of the angular velocity for the first propellr and',
      ' decrease of the angular velocity of the second propellr by %f.2f'%math.sqrt(0.9),' will result in',
      '%5.2f'%ang_acceleration, 'rad/(s*s) angular acceleration.' )

Answers.angular_acceleration(bi.i_z, bi.k_m, bi.omega_1, bi.omega_2, ang_acceleration)


# [Solution](/notebooks/1.%20Coaxial%20Drone%20Dynamics%20SOLUTION.ipynb)

# In[ ]:


class Drone2D:

    def __init__(self,
                 k_f=0.1,  # value of the thrust coefficient
                 I_x=0.1,  # moment of inertia around the x-axis
                 m=1.0,  # mass of the vehicle
                 l=0.5,  # distance between the center of
                 #   mass and the propeller axis
                 ):
        self.k_f = k_f
        self.I_x = I_x
        self.l = l
        self.m = m

        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = 9.81

        # z, y, phi, z_dot, y_dot, phi_dot
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def advance_state_uncontrolled(self, dt):
        """Advances the state of the drone by dt seconds.
        Note that this method assumes zero rotational speed
        for both propellers."""

        X_dot = np.array([
            self.X[3],
            self.X[4],
            self.X[5],
            self.g,
            0.0,
            0.0])
        # Change in state will be
        self.X = self.X + X_dot * dt
        return self.X

    # NOTE - this is a new helper function which you may find useful
    def get_thrust_and_moment(self):
        """Helper function which calculates and returns the
        collective thrust and the moment about the X axis"""

        f1 = self.k_f * self.omega_1 ** 2
        f2 = self.k_f * self.omega_2 ** 2

        # c is often used to indicate "collective" thrust
        c = f1 + f2

        M_x = (f1 - f2) * self.l
        return c, M_x

    ##################################
    # BEGIN TODOS ####################

    @property
    def z_dot_dot(self):
        """Calculates vertical (z) acceleration of drone."""

        # TODO 1
        #  Calculate the vertical component of the acceleration
        #  You might find get_thrust_and_moment helpful
        c, M = self.get_thrust_and_moment()

        return self.g - c * np.cos(self.X[2])

    @property
    def y_dot_dot(self):
        """Calculates lateral (y) acceleration of drone."""

        # TODO 2
        #  Calculate the horizontal component of the acceleration
        c, M = self.get_thrust_and_moment()

        return c * np.sin(self.X[2])

    @property
    def phi_dot_dot(self):
        # TODO 3
        #  Calculate the angular acceleration about the x-axis.
        c, M = self.get_thrust_and_moment()

        return M / self.I_x