import numpy as np

# time step
dt = 1.0

# x_hat = [position, velocity]
x = np.array([0.0, 1.0])

# covariance matrix
P = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])

# prediction matrix for position and velocity
# using kinematic equation Pk = Pk1 + dt * Vl1
F = np.array([
    [1.0, dt],
    [0.0, 1]
])

# update prediction
x = F.dot(x)
# update covariance matrix
P = np.matmul(F, np.matmul(P, F.transpose()))
print(x, P)

# update prediction
x = F.dot(x)
# update covariance matrix
P = np.matmul(F, np.matmul(P, F.transpose()))
print(x, P)
