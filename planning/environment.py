import numpy as np
from scipy.spatial import Voronoi
from bresenham import bresenham

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
    print(0, east_max - east_min)

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    print(data.shape[0])
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        # Determine which cells contain obstacles
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
        for e in range(x0, xm):
            for n in range(y0, ym):
                # skip out of range conditions
                if e < 0:
                    continue
                if e >= em:
                    continue
                if n < 0:
                    continue
                if n >= nm:
                    continue
                if (alt + d_alt + safety_distance) <= drone_altitude:
                    continue
                # plot it
                grid[n][e] = 1

    return grid


def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Define a list to hold Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min),
                int(north + d_north + safety_distance - north_min),
                int(east - d_east - safety_distance - east_min),
                int(east + d_east + safety_distance - east_min),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)
    # check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]].astype(int)
        p2 = graph.vertices[v[1]].astype(int)
        # test each pair p1 and p2 for collision using Bresenham
        # If the edge does not hit an obstacle add it to the list
        in_collision = False
        ridgeline = bresenham(p1[0], p1[1], p2[0], p2[1])
        for b in ridgeline:
            # eliminate out of range points in the line
            if b[0] < 0 or b[0] >= grid.shape[0]:
                in_collision = True
                break
            if b[1] < 0 or b[1] >= grid.shape[1]:
                in_collision = True
                break
            # check if grid cell is an obstacle
            if grid[b[0], b[1]] == 1:
                in_collision = True
                break
        # keep ridge points not in collision
        if not in_collision:
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges
