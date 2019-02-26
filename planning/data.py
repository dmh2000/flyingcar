import numpy as np


def colliders(filename=None):
    # This is the same obstacle data from the previous lesson.
    filename = 'colliders.csv'
    csv_data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
    return csv_data



