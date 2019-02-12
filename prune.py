def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    if path is None:
        return path
    if len(path) < 3:
        return path
    
    pruned_path = []
    p1 = path[0]
    pruned_path.append(p1)
    p2 = path[1]
    for p in path[2:]:
        p3 = p
        if collinearity_check(point(p1),point(p2),point(p3)):
            # skip p1
            p2 = p3
        else:
            pruned_path.append(p2)
            p1 = p2
            p2 = p3
            
    return pruned_path    