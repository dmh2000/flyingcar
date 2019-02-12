import numpy as np
from queue import PriorityQueue
from enum import Enum
import math
# =======================================
# ASTAR GRAPH
# =======================================


def make_node(pos, cost):
    return (pos, cost)


# modify A* to work with a graph
def a_star_graph(G, h, start, goal):
    # queue and branch nodes are tuple of ((x,y),cost)
    path = []
    path_cost = 0
    queue = PriorityQueue()
    start_node = make_node(start, 0.0)
    # print(start_node)
    queue.put(start_node)
    visited = set(start_node)

    branch = {}
    found = False

    while not queue.empty():
        current_node = queue.get()
        node_pos = current_node[0]

        # set current cost
        if np.allclose(node_pos, start):
            current_cost = 0.0
        else:
            current_cost = branch[node_pos][0]

        # if close to goal, quit
        if np.allclose(node_pos, goal):
            print('Found a path.')
            found = True
            break
        else:
            # for each node adjacent to current_node
            for next_node in G[current_node[0]]:
                # get the tuple representation
                next_node = make_node(next_node, G.edges[current_node[0], next_node]['weight'])
                # sum the current path cost
                branch_cost = current_cost + next_node[1]
                # add the heuristic cost
                queue_cost = branch_cost + h(next_node[0], goal)

                if next_node[0] not in visited:
                    visited.add(next_node[0])
                    branch[next_node[0]] = (branch_cost, current_node[0])
                    # print(next_node[0], ' --> ', current_node[0])
                    queue.put(make_node(next_node[0], queue_cost))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while not np.allclose(n, start):
            path.append(n)
            n = branch[n][1]
        path.append(n)
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost

# =======================================
# ASTAR GRID
# =======================================


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return self.value[0], self.value[1]


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)

    return valid


def a_star_grid(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                # TODO: calculate branch cost (action.cost + g)
                # TODO: calculate queue cost (action.cost + g + h)
                branch_cost = action.cost + current_cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost
