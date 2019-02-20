# net torque = tau = I * alpha
# alpha = angular acceleration radians/sec**2
# I = moment of inertia kg * m**2
# Mz = Iz * psi_dot_dot (angular acceleration)

m = 1.0  # mass
Iz = 2.0
Kf = 0.1
Km = 0.2
O1 = 10.0  # radians/sec CW
O2 = 8.0  # radian/sec CCW
G = -9.8

M1 = Iz *  O1
M2 = Iz * -O2
O  = O1 + O2
M  = Km * (O2**2 - O1**2) # negative omega (CCW) - positive omega (CW)
O  = M / 2.0

print(M , O)

# f = ma
# a = m / f
Fz1 =  Kf * O1**2
Fz2 =  Kf * O2**2
Fz = Fz1 + Fz2 + G
print(Fz, Fz1, Fz2)
