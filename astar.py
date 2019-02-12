import numpy as np
from queue import PriorityQueue

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
        i = 0
        while not np.allclose(n, start):
            path.append(n)
            n = branch[n][1]
        path.append(n)
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost
