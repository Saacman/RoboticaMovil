import numpy as np
import pickle
import matplotlib.pyplot as plt

#          x, y, z
delta = [[-1, 0, 0], # back
         [ 0,-1, 0], # left
         [ 1, 0, 0], # front
         [ 0, 1, 0], # right
         [ 0, 0,-1], # down
         [ 0, 0, 1]] # up
cost = 1


def getHeuristic(grid,goal):
    xdim=grid.shape[0]
    ydim=grid.shape[1]
    zdim=grid.shape[2]
    heuristic = np.zeros((xdim,ydim,zdim), dtype=np.float32)
    for z in range(zdim):
        for y in range(ydim):
            for x in range(xdim):
                # Euclidean Distance
                dist=((x-goal[0])**2+(y-goal[1])**2+(z-goal[2])**2)**(1/2.0)
                yheu = np.abs(float(y) - goal[1])
                heuristic[x,y,z]= dist + yheu

    return heuristic


def search(init, goal, grid, heuristic, maxp):
    xdim=grid.shape[0]
    ydim=grid.shape[1]
    zdim=grid.shape[2]
    x = init[0]
    y = init[1]
    z = init[2]
    
    closed = np.zeros((xdim,ydim,zdim), dtype=np.int8)
    closed[x,y,z] = 1

    expand = np.ones((xdim,ydim,zdim), dtype=np.int8)*-1
    action = np.ones((xdim,ydim,zdim), dtype=np.int8)*-1

    g = 0
    h = heuristic[x,y,z]
    f = g+h

    openl = [[f, g, x, y, z]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
  
    while not found and not resign and count < 1e6:
        if len(openl) == 0:
            resign = True
            return "Fail: Open List is empty"
        else:
            openl.sort()
            openl.reverse()
            nextl = openl.pop()
            
            x = nextl[2]
            y = nextl[3]
            z = nextl[4]
            g = nextl[1]
            f = nextl[0]
            expand[x,y,z] = count
            count += 1

            if x == goal[0] and y == goal[1] and z == goal[2]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    z2 = z + delta[i][2]
                    
                    if z2 >= 0 and z2 < zdim and \
                        y2 >=0 and y2 < ydim and \
                        x2 >=0 and x2 < xdim:
                            
                            if closed[x2,y2,z2] == 0 and grid[x2,y2,z2] < maxp:

                                g2 = g + cost
                                f2 = g2 + heuristic[x2,y2,z2]
                                openl.append([f2, g2, x2, y2, z2])
                                closed[x2,y2,z2] = 1
                                # Memorize the sucessfull action for path planning
                                action[x2,y2,z2] = i
                    else:
                        pass

    path=[]
    path.append([goal[0], goal[1], goal[2]])
    
    while x != init[0] or y != init[1] or z != init[2]:
        x2 = x-delta[action[x,y,z]][0]
        y2 = y-delta[action[x,y,z]][1]
        z2 = z-delta[action[x,y,z]][2]
        
        x = x2
        y = y2
        z = z2
        # Path
        path.append([x2, y2, z2])

    path.reverse()
    return path

