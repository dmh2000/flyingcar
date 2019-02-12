#!/usr/bin/env python
# coding: utf-8

# ## Finding Your Way In The City (Graph Edition)
# In this notebook your attention will shift from grids to graphs. At least for search ... 
# 
# Using Voronoi graphs and the medial axis transform we can find paths which maximize safety from obstacles. In addition, graph representation allows further optimizations and more succinct queries.

# In[166]:


# OK this might look a little ugly but...
# need to import the latest version of networkx
# This occassionally fails, so if the next cell 
# doesn't show that you're using networkx 2.1
# please "restart and clear output" from "Kernel" menu
# above and try again.
import sys
get_ipython().system('{sys.executable} -m pip install -I networkx==2.1')
# import pkg_resources
# pkg_resources.require("networkx==2.1")


# In[167]:


import networkx as nx
nx.__version__


# In[168]:


import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid_and_edges
import numpy.linalg as LA
get_ipython().run_line_magic('matplotlib', 'inline')


# In[169]:


plt.rcParams['figure.figsize'] = 12, 12


# In[170]:


# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# Starting and goal positions in *(north, east)*.

# In[171]:


start_ne = (25,  100)
goal_ne = (750., 370.)


# In[172]:


# Static drone altitude (metres)
drone_altitude = 5
safety_distance = 3


# In[173]:


# This is now the routine using Voronoi
grid, edges = create_grid_and_edges(data, drone_altitude, safety_distance)
# print(len(edges))


# Plot the edges on top of the grid along with start and goal locations.

# In[174]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower', cmap='Greys') 

for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# We now have a graph, well at least visually. The next step is to use the [`networkx`](https://networkx.github.io) to create the graph. **NetworkX** is a popular library handling anything and everything related to graph data structures and algorithms.
# 
# **NOTE:** In the initial import above it was imported with the `nx` alias.
# 
# You're encouraged to read the documentation but here's a super quick tour:
# 
# 1. Create a graph:
# 
# ```
# G = nx.Graph()
# ```
# 
# 2. Add an edge:
# 
# ```
# p1 = (10, 2.2)
# p2 = (50, 40)
# G = nx.add_edge(p1, p2)
# ```
# 
# 3 Add an edge with a weight:
# 
# ```
# p1 = (10, 2.2)
# p2 = (50, 40)
# dist = LA.norm(np.array(p2) - np.array(p1))
# G = nx.add_edge(p1, p2, weight=dist)
# ```

# In[175]:


# create the graph with the weight of the edges
# set to the Euclidean distance between the points
G = nx.Graph()

for e in edges:
    dist = LA.norm(np.array(e[0]) - np.array(e[1]))
    G.add_edge(e[0],e[1],weight=dist)

#for n in G:
#    for adj in G[n]:
#        print(n)
#        print('   ',(adj[0],adj[1],G.edges[n,adj]['weight']))
        


# You need a method to search the graph, and you'll adapt A* in order to do this. The notable differences being the actions are now the outgoing edges and the cost of an action is that weight of that edge.

# In[184]:


from queue import PriorityQueue

def make_node(pos,cost):
    return (pos,cost)

#modify A* to work with a graph
def a_star(G, h, start, goal):
    # queue and branch nodes are tuple of ((x,y),cost)
    path = []
    path_cost = 0
    queue = PriorityQueue()
    start_node = make_node(start,0.0)
    # print(start_node)
    queue.put(start_node)
    visited = set(start_node)

    branch = {}
    found = False
    
    while not queue.empty():
        current_node = queue.get()
        node_pos  = current_node[0]
        node_cost = current_node[1]
        
        # set current cost
        if np.allclose(node_pos,start):
            current_cost = 0.0
        else:           
            current_cost = branch[node_pos][0]
            
        # if close to goal, quit
        if np.allclose(node_pos,goal):
            print('Found a path.')
            found = True
            break
        else:
            # for each node adjacent to current_node
            for next_node in G[current_node[0]]:
                # get the tuple representation
                next_node = make_node(next_node,G.edges[current_node[0],next_node]['weight'])
                # sum the current path cost
                branch_cost = current_cost + next_node[1]
                # add the heuristic cost
                queue_cost = branch_cost + h(next_node[0], goal)
                
                if next_node[0] not in visited:                
                    visited.add(next_node[0])    
                    branch[next_node[0]] = (branch_cost, current_node[0])
                    # print(next_node[0], ' --> ', current_node[0])
                    queue.put(make_node(next_node[0],queue_cost))
             
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


# ### Solution
# 
# This solution consists of two parts:
# 
# 1. Find the closest point in the graph to our current location, same thing for the goal location.
# 2. Compute the path from the two points in the graph using the A* algorithm.
# 3. Feel free to use any of the path pruning techniques to make the path even smaller! 
# 4. Plot it up to see the results!

# ### TODO: Write your solution!

# [our solution](/notebooks/Graph-Search-Solution.ipynb)

# In[185]:


# Euclidian distance
def euclidian_distance(position, goal_position):
    x = goal_position[0] - position[0]
    y = goal_position[1] - position[1]
    h = np.sqrt(x*x + y*y)
    
    return h

def norm_distance(position,goal_position):
    return LA.norm(np.array(position) - np.array(goal_position))

def find_nearest(G,pos):
    mind = 1000000
    p = None
    for n in G:
        a = np.array(n)
        b = pos
        d = LA.norm(a-b)
        if d < mind:
            mind = d
            p = a
            
    return (p[0],p[1])

# find start node from start_ne
start = find_nearest(G,start_ne)
# find goal node from  goal_ne
goal  = find_nearest(G,goal_ne)

print(start,start_ne)
print(goal,goal_ne)

#path,cost = a_star(G,euclidian_distance,start,goal)
path,cost = a_star(G,norm_distance,start,goal)
print(len(path))
print(cost)


# In[ ]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
# Plot it up!
plt.imshow(grid, origin='lower', cmap='Greys') 

for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
    
# Stepping through each edge
p1 = path[0]
for p in path[1:]:
    p2 = p
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
    p1 = p2

plt.plot(start[1], start[0], 'rx')
plt.plot(goal[1], goal[0], 'rx')
plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# In[ ]:





# In[ ]:




