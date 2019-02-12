# Define your breadth-first search function here
def breadth_first(grid, start, goal):

    # TODO: Replace the None values for 
        # "queue" and "visited" with data structure objects
        # and add the start position to each 
    q = Queue()     # create an empty queue
    visited = set() # create an empty set
    branch = {}     # create an empty dictionary
    found = False
    
    # initialize start position
    q.put(start)
    visited.add(start)
    
    # Run loop while queue is not empty
    while not q.empty(): # TODO: replace True with q.empty():
        # remove the first element from the queue
        current_node = q.get()
        
        # check if the current node corresponds to the goal state
        if current_node == goal: 
            print('Found a path.')
            found = True
            break
        else:
            # Get the new nodes connected to the current node
            # Iterate through each of the new nodes and:
            # If the node has not been visited you will need to
            # 1. Mark it as visited
            # 2. Add it to the queue
            # 3. Add how you got there to the branch dictionary
            actions = valid_actions(grid,current_node)
            for action in actions:
                next_node = (current_node[0] + action.value[0],current_node[1]+action.value[1])
                if next_node not in visited:
                    visited.add(next_node)
                    q.put(next_node)
                    branch[next_node] = (current_node,action)
            
    
    # Now, if you found a path, retrace your steps through 
    # the branch dictionary to find out how you got there!
    path = []
    if found:
        # retrace steps
        path = []
        n = goal
        while branch[n][0] != start:
            path.append(branch[n][1])
            n = branch[n][0]
        path.append(branch[n][1])
            
    return path[::-1]