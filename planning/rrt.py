import numpy as np
import matplotlib.pyplot as plt
import sklearn
from sklearn.neighbors import KDTree
import networkx as nx


class RRT:
    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)

    def add_vertex(self, x_new):
        self.tree.add_node(tuple(x_new))

    def add_edge(self, x_near, x_new, u):
        self.tree.add_edge(tuple(x_near), tuple(x_new), orientation=u)

    @property
    def vertices(self):
        return list(self.tree)
        # return self.tree.nodes

    @property
    def edges(self):
        return self.tree.edges()


def sample_state(grid):
    # get a random sample in the range of the grid
    minx = 0
    maxx = grid.shape[0]
    miny = 0
    maxy = grid.shape[1]

    # repeat until sample is not in an obstructino
    x = int(np.random.uniform(minx, maxx))
    y = int(np.random.uniform(miny, maxy))
    return x, y


def nearest_neighbor(x_rand, rrt):
    # create a KDtree of current points in the rrt
    # query it for nearest point
    v = rrt.vertices
    tree = KDTree(v)
    idx = tree.query([x_rand], k=1, return_distance=False)[0][0]
    v = rrt.vertices[idx]
    return v


def select_input(x_rand, x_near):
    angle = np.arctan2(x_rand[1] - x_near[1], x_rand[0] - x_near[0])
    return angle


def new_state(grid, x_near, u, dt):
    x1 = int(x_near[0] + np.cos(u) * dt)
    y1 = int(x_near[1] + np.sin(u) * dt)
    # return near point instead of going forward
    if grid[x1, y1] == 1:
        x1 = x_near[0]
        y1 = x_near[1]

    return [x1, y1]


def generate_rrt(grid, x_init, num_vertices, dt):
    rrt = RRT(x_init)

    for _ in range(num_vertices):

        x_rand = sample_state(grid)
        # sample states until a free state is found
        while grid[int(x_rand[0]), int(x_rand[1])] == 1:
            x_rand = sample_state(grid)

        x_near = nearest_neighbor(x_rand, rrt)

        u = select_input(x_rand, x_near)
        x_new = new_state(grid,x_near, u, dt)

        if grid[int(x_new[0]), int(x_new[1])] == 0:
            # the orientation `u` will be added as metadata to
            # the edge
            rrt.add_edge(x_near, x_new, u)

    return rrt


def create_grid():
    grid = np.zeros((100, 100))
    # build some obstacles
    grid[10:20, 10:20] = 1
    grid[63:80, 10:20] = 1
    grid[43:60, 30:40] = 1
    grid[71:86, 38:50] = 1
    grid[10:20, 55:67] = 1
    grid[80:90, 80:90] = 1
    grid[75:90, 80:90] = 1
    grid[30:40, 60:82] = 1
    return grid


print(np.__version__)
print(sklearn.__version__)
print(nx.__version__)
num_vertices = 300
dt = 1
x_init = (50, 50)

grid = create_grid()

rrt = generate_rrt(grid, x_init, num_vertices, dt)

plt.imshow(grid, cmap='Greys', origin='lower')
plt.plot(x_init[1], x_init[0], 'ro')

for (v1, v2) in rrt.edges:
    plt.plot([v1[1], v2[1]], [v1[0], v2[0]], 'y-')

plt.show()


