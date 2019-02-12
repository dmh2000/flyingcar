def uniform_cost(grid, start, goal):

    # TODO: Initialize the starting variables
    path    = []
    queue   = PriorityQueue()
    visited = set()
    
    branch = {}
    found = False
    
    queue.put( (0, start))
    visited.add(start)
    
    while not queue.empty():
        # Remove the first element from the queue
        v = queue.get()
        current_cost = v[0]
        current_node = v[1]
        # TODO: Check if the current vertex corresponds to the goal state
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # determine the next_node using the action delta
                next_node = (current_node[0] + action.delta[0],current_node[1] + action.delta[1])
                # compute the new cost
                new_cost = current_cost + action.cost
                
                # Check if the new vertex has not been visited before.
                # If the node has not been visited you will need to
                # 1. Mark it as visited
                # 2. Add it to the queue
                if next_node not in visited:
                    visited.add(next_node)
                    queue.put( (new_cost, next_node) )
                    branch[next_node] = (new_cost, current_node, action)
             
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])
            
    return path[::-1], path_cost