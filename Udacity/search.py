#Search. path planning

# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    
    closed = [0 for row in range(len(grid[0]))] for col in range(leng(grid[
    closed[init[0]][init[1]] = 1 #start is checked

    #intial values
    x = init[0]
    y = init[1]
    g = 0
    
    open_list = [[g,init]]
    print 'initial list item'
    initialCost = 0
    open_list.insert(0,initialCost)
    print open_list
    print 'new open list:'
    #open_list[0] = open_list[0]+1 #cost updated
    
    '''
    check grids to the left, right, up and down of currrent node
    if not = 1 (not an obstacle)
    add to open list items
    find node with lowest cost value
    '''

    #remove node with lowest cost
    open_list.sort()
    open_list.reverse() #to get smallest node cost
    next = open_list.pop()
    x = next[1]
    y = next[2]
    g = next[0]

    #check to see if at goal
    if x == goal[0] and y == goal[1]:
        found = True
        print 'Found Goal state'
    
    '''
    #down shift
    shiftD1 = x + delta[2][0]
    shiftD2 = y + delta[2][1]
    if (grid[shiftD1][shiftD2] != 1 and shiftD1 >= 0 and shiftD2 >= 0):
        open_list[0] = open_list[0]+1
        open_list[1:] = shiftD1,shiftD2
    print open_list
    '''
    #all four moves at once
    for i in range(len(delta)):
        x2 = x + delta[i][0]
        y2 = y + delta[i][1]
        if (x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 > len(grid[0])):
            if closed[x2][y2] == 0 and grid[x2][y2] == 0:
               g2 = g + cost
               open_list.append([g2,x2,y2])
               closed[x2][y2] = 1 #don't search on this node again           
   
    path = open_list
    
    '''
    take list item
    [0,0,0]
    new open list:
    [1,1,0]
    [1,0,1]
    take list item
    [1,0,1]
    new open list:
    [1,1,0]
    [2,1,1]
    '''
    
    return path

goal = search(grid,init,goal,cost)
print goal

#expansion grid.

    

