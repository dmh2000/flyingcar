import matplotlib.pyplot as plt
import networkx as nx

from fcar.planning import environment, data, astar, utils


def create_graph(data, drone_altitude, safety_distance):
    """
    create a grid and graph for astar planning
    :param data:
    :param drone_altitude:
    :param safety_distance:
    :return: populated grid , edges (for display) and voronoi graph though open paths
    """
    grid, edges = environment.create_grid_and_edges(data, drone_altitude, safety_distance)
    print('Found %5d edges' % len(edges))

    # =====================================
    # create graph
    # =====================================
    # create the graph with the weight of the edges
    # set to the Euclidean distance between the points
    G = nx.Graph()

    for e in edges:
        dist = utils.euclidian_distance(e[0], e[1])  # LA.norm(np.array(e[0]) - np.array(e[1]))
        G.add_edge(e[0], e[1], weight=dist)

    return G, edges, grid


def local_graph(graph, p1=None, p2=None):
    """
    add any new obstacle to subset of grid
    :param graph: input grid
    :param p1: start point
    :param p2: end point
    :return:  updated grid
    """
    return graph


def local_path(G, grid, lcl, p1, p2):
    """create local path from p1 to p2"""

    # create a local graph
    lcl_graph = local_graph(G)

    # plan over the local graph from p1 to p2
    path, _ = astar.a_star_graph(lcl_graph, utils.norm_distance, p1, p2)
    print(path)
    for p in path:
        lcl.append(p)
    return lcl


def show_grid(grid, edges, path1, path2):
    """
    plot the grid with edges and path (if they exist)
    :param grid: 2D grid
    :param edges: networx graphs edges or None
    :param path1: array of points in path or None
    :param path2: array of points in path or None
    :return: none
    """
    plt.imshow(grid, origin='lower', cmap='Greys')

    if edges is not None:
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    # Stepping through each edge
    if path1 is not None:
        p1 = path1[0]
        for p in path1[1:]:
            p2 = p
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'ro')
            p1 = p2

    if path2 is not None:
        p1 = path2[0]
        for p in path2[1:]:
            p2 = p
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'g-', linewidth=3)
            p1 = p2

    plt.plot(start[1], start[0], 'rx')
    plt.plot(goal[1], goal[0], 'rx')
    plt.grid()
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.xticks([x for x in range(0, 1000, 50)])
    plt.yticks([y for y in range(0, 1000, 50)])
    plt.show()


# %matplotlib inline
plt.rcParams['figure.figsize'] = 14, 14

# =====================================
# get colliders
# =====================================
data = data.colliders()
print("data :", len(data))

# grid without edges
# flight_altitude = 5
# safety_distance = 3
# grid = utils.create_grid(data, flight_altitude, safety_distance)

# =====================================
# create grid with edges
# =====================================
drone_altitude = 5
safety_distance = 3

#grid, edges = environment.create_grid_and_edges(data, drone_altitude, safety_distance)
#print('Found %5d edges' % len(edges))

# =====================================
# create graph
# =====================================
# create the graph with the weight of the edges
# set to the Euclidean distance between the points
#G = nx.Graph()

#for e in edges:
#    dist = utils.euclidian_distance(e[0], e[1])  # LA.norm(np.array(e[0]) - np.array(e[1]))
#    G.add_edge(e[0], e[1], weight=dist)

G, edges, grid = create_graph(data,drone_altitude,safety_distance)

# =====================================
# get start/goal
# =====================================
start_ne = (25, 100)
goal_ne = (750., 370.)

# find start node from start_ne
start = utils.find_nearest(G, start_ne)
# find goal node from  goal_ne
goal = utils.find_nearest(G, goal_ne)

print(start, start_ne)
print(goal, goal_ne)

# path,cost = a_star(G,euclidian_distance,start,goal)
path, cost = astar.a_star_graph(G, utils.norm_distance, start, goal)
print(path)

# new path
new_path = []

# create coarse path
path = [path[i] for i in range(0, len(path), 20)]

# go to each point in path
# add start position
new_path.append(path[0])
p1 = new_path[0]
for p2 in path[1:]:
    # add local path to existing path
    new_path = local_path(G, grid, new_path, p1, p2)
    p1 = p2
new_path.append(goal)

# show grid
show_grid(grid, edges, path, new_path)
