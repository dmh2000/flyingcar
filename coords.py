import numpy as np
from enum import Enum

class Rotation(Enum):
    ROLL = 0
    PITCH = 1
    YAW = 2  


class EulerRotation:
    
    def __init__(self, rotations):
        """
        `rotations` is a list of 2-element tuples where the
        first element is the rotation kind and the second element
        is angle in degrees.
        
        Ex:
        
            [(Rotation.ROLL, 45), (Rotation.YAW, 32), (Rotation.PITCH, 55)]
            
        """
        self._rotations = rotations
        self._rotation_map = {Rotation.ROLL : self.roll, Rotation.PITCH : self.pitch, Rotation.YAW : self.yaw}

    def roll(self, phi):
        """Returns a rotation matrix along the roll axis"""
        phi = phi * (np.pi / 180.0)
        m = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi),  np.cos(phi)]
            ])
        return m
    
    def pitch(self, theta):
        """Returns the rotation matrix along the pitch axis"""
        theta = theta * (np.pi / 180.0)
        m = np.array([
            [ np.cos(theta), 0,  np.sin(theta)],
            [ 0            , 1,  0],
            [-np.sin(theta), 0,  np.cos(theta)]
            ])
        return m

    def yaw(self, psi):
        """Returns the rotation matrix along the yaw axis"""
        psi = psi * (np.pi / 180.0)
        m = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi),  np.cos(psi), 0],
            [0          ,  0          , 1]
            ])
        return m

    def rotate(self):
        """Applies the rotations in sequential order"""
        # reverse the apparent order in the code
        # to match the matrix multiplication conventions
        t = np.eye(3)
        for r in self._rotations:
            # select the function
            f = self._rotation_map[r[0]]
            # get the angle
            a = r[1]
            # multiply the matrix
            t = np.dot(f(a), t)
            #np.dot(self._rotation_map[self._rotations[2][0]](self._rotations[2][1]),
            #       self._rotation_map[self._rotations[1][0]](self._rotations[1][1])),
            #       self._rotation_map[self._rotations[0][0]](self._rotations[0][1]))
        return t

def euler_to_quaternion(angles):
    roll = angles[0]
    pitch = angles[1]
    yaw = angles[2]
    
    # complete the conversion
    # and return a numpy array of
    # 4 elements representing a quaternion [a, b, c, d]
    cos_psi = np.cos(yaw / 2.0)
    sin_psi = np.sin(yaw / 2.0)
    cos_phi = np.cos(roll / 2.0)
    sin_phi = np.sin(roll / 2.0)
    cos_tht = np.cos(pitch / 2.0)
    sin_tht = np.sin(pitch / 2.0)
    
    a = np.array([
        cos_phi * cos_tht * cos_psi + sin_phi * sin_tht * sin_psi,
        sin_phi * cos_tht * cos_psi - cos_phi * sin_tht * sin_psi,
        cos_phi * sin_tht * cos_psi + sin_phi * cos_tht * sin_psi,
        cos_phi * cos_tht * sin_psi - sin_phi * sin_tht * cos_psi
    ])
    return a.flatten();

def quaternion_to_euler(quaternion):
    a = quaternion[0]
    b = quaternion[1]
    c = quaternion[2]
    d = quaternion[3]
    
    # complete the conversion
    # and return a numpy array of
    # 3 element representing the euler angles [roll, pitch, yaw]
    y = 2 * (a * b + c * d)
    x = 1 - 2 * (b * b + c * c)
    phi = np.arctan2(y,x)
    theta = np.arcsin(2 * (a * c - d * b))
    y = 2 * (a*d + b*c)
    x = 1- 2 * (c*c + d*d)
    psi = np.arctan2(y,x)
    return np.array([phi,theta,psi])



def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))
    print(0, north_max - north_min)

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))
    print(0,east_max-east_min)

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        # TODO: Determine which cells contain obstacles
        # and set them to 1.
        #
        # Example:
        #
        #    grid[north_coordinate, east_coordinate] = 1
        nc = int(north - north_min)
        ec = int(east - east_min)
        dn = int(d_north)
        de = int(d_east)
        sd = int(safety_distance)
        x0 = int(ec - (de + sd))
        y0 = int(nc - (dn + sd))
        xm = int(ec + (de + sd))
        ym = int(nc + (dn + sd))
        nm = north_max - north_min
        em = east_max - east_min
        print(drone_altitude,alt,d_alt)
        for e in range(x0,xm):
            for n in range(y0,ym):
                # skip out of range conditions
                if e < 0: 
                    continue
                if e >= em:
                    continue
                if n < 0:
                    continue
                if n >= nm:
                    continue
                # check if drone is above obstacle altitude
                if alt + d_alt + safety_distance > drone_altitude:
                    continue
                # plot it
                grid[n][e] = 1
        
    return grid
    