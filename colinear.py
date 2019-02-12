import numpy as numpy

# determinate of matrix == 0 means points are colinear
COLINEAR_EPSILON = 0.1

def colinear2D(p1,p2,p3):
  area = p1[0] * (p2[1] - p3[1]) + \
         p2[0] * (p3[1] - p1[1]) + \
         p3[0] * (p1[1] - p2[1])
  return np.abs(area) < COLINEAR_EPSILON


"""
discretize the world
use a* to find path
test for colinearity and remove cells
"""
def add_one(p):
    p = np.array([p[0],p1[1],1])
    return p


def collinearity_float(p1, p2, p3, epsilon=1e-2): 
    collinear = False
    epsilon = .01
    # TODO: Add a third dimension of z=1 to each point
    p1 = add_one(p1)
    p2 = add_one(p2)
    p3 = add_one(p3)
    # TODO: Create the matrix out of three points
    m = np.vstack([p1,p2,p3])
    print(m)
    # TODO: Calculate the determinant of the matrix. 
    d = np.linalg.det(m)
    # TODO: Set collinear to True if the determinant is less than epsilon
    if abs(d) < epsilon:
        collinear = True

    return collinear

    def collinearity_int(p1, p2, p3): 
    collinear = False
    epsilon = 10
    # TODO: Calculate the determinant of the matrix using integer arithmetic 
    p1 = [int(p1[0]) * 100,int(p1[1])*100]
    p2 = [int(p2[0]) * 100,int(p2[1])*100]
    p3 = [int(p3[0]) * 100,int(p3[1])*100]
    
    area = p1[0] * (p2[1] - p3[1]) + \
           p2[0] * (p3[1] - p1[1]) + \
           p3[0] * (p1[1] - p2[1])
    

    print(area)
    # TODO: Set collinear to True if the determinant is equal to zero
    if abs(area) < epsilon:
        collinear = True

    return collinear

    def rtrace(p1,p2,x):
      return np.floor((p2[1] - p1[1]) / (p2[0] - p1[0])) * x + p1[1]

    def bresenham(p1,p2):
      # slope
      x = p1[0]
      y = p1[1]
      m = (p2[1] - p1[1]) / (p2[0] - p1[0]) # slope
      if (y + m) > (y + ):
        x += 1
        y += 1
      else:
        x += 1
        y += 0

     

def bres1(p1, p2): 
    """
    Note this solution requires `x1` < `x2` and `y1` < `y2`.
    """
    x1, y1 = p1
    x2, y2 = p2
    cells = []
    
    # Determine valid grid cells
    
    # compute slope
    m = (y2 - y1) / (x2 - x1)
    # starting points
    x = x1
    y = y1
    # add first cell before continuing
    cells.append((x1,y1))
    # while current x is less than endpoint
    while x < x2:
        # get current line position
        f = (x+1) * m
        # check what to increment
        if f > (y+1):
            y += 1
        else:
            x += 1
        cells.append((x,y))
    return np.array(cells)

def bres2(p1, p2): 
    """
    Note this solution requires `x1` < `x2` and `y1` < `y2`.
    """
    x1, y1 = p1
    x2, y2 = p2
    cells = []
    
    # Determine valid grid cells
    
    # compute slope
    m = (y2 - y1) / (x2 - x1)
    # starting points
    x = x1
    y = y1
    # add first cell before continuing
    cells.append((x1,y1))
    # while current x is less than endpoint
    while x < x2:
        # get current line position
        f = (x+1) * m
        # check what to increment
        if f > (y+1):
            y += 1
            cells.append((x,y))
        elif f == (y+1):
            x += 1
            cells.append((x-1,y+1))
            cells.append((x-1,y-1))
            cells.append((x,y-1))
            cells.append((x,y))
            cells.append((x,y+1))
        else:
            x += 1
            cells.append((x,y))

    return np.array(cells)
