import numpy as np

# nonlinear system of equations
# x -> position of cart
# theta -> angle of pendulum
# X = vector of states =  [x,x_dot,theta,theta_dot]
# fixed pointss :
#    theta = 0 (down), PI(up)
#    theta_dot = 0
#    x_dot = 0
#    x is a free variable

# u = force on cart in x direction
