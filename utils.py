import numpy as np
import numpy.linalg as LA
import networkx

# Euclidian distance
def euclidian_distance(position, goal_position):
    x = goal_position[0] - position[0]
    y = goal_position[1] - position[1]
    h = np.sqrt(x * x + y * y)

    return h


def norm_distance(position, goal_position):
    return LA.norm(np.array(position) - np.array(goal_position))


def find_nearest(G, pos):
    mind = 1000000
    p = None
    for n in G:
        a = np.array(n)
        b = pos
        d = LA.norm(a - b)
        if d < mind:
            mind = d
            p = a

    return (p[0], p[1])



