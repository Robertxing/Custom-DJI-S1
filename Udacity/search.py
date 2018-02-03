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

# -----------
# User Instructions:
#
# Modify the the search function so that it returns
# a shortest path as follows:
# 
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left, 
# up, and down motions. Note that the 'v' should be 
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.
# ----------

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
    
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1 #start is checked

    #intial values
    x = init[0]
    y = init[1]
    g = 0
    expandCounter = 0
    expand = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    path = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]

    expand[init[0]][init[1]] = expandCounter
    open_list = [[g,x,y]]
    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    
    #print 'initial list item'
    #initialCost = 0
    #open_list.insert(0,initialCost)
    #print open_list
    #print 'new open list:'
    #open_list[0] = open_list[0]+1 #cost updated
    
    while found is False and resign is False:
        if len(open_list) == 0:
            resign = True
            print 'fail'
        else:
            #remove node with lowest cost
            #print 'open list: '
            #print open_list
            open_list.sort()
            open_list.reverse() #to get smallest node cost
            nextNode = open_list.pop()
            #print 'current node: '
            #print nextNode
            x = nextNode[1]
            y = nextNode[2]
            g = nextNode[0]
            expand[x][y] = expandCounter
            #expand[x][y] = ' '
            expandCounter += 1             

            #check to see if at goal
            if (x == goal[0] and y == goal[1]):
                found = True
                print 'Found Goal state'
                print nextNode
                expand[x][y] =  '*'
            else:
                #all four moves at once
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if (x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0])):
                        #if not checked before and not obstacle
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost                            
                            open_list.append([g2,x2,y2])                            
                            closed[x2][y2] = 1 #don't search on this node again
                            action[x2][y2] = i #link chosen action to successor state

            x = goal[0]
            y = goal[1]
        while (x != init[0] and y != init[1]):
            x2 = x - delta[action[x][y]][0]
            y2 = y - delta[action[x][y]][1]
            path[x2][y2] = delta_name[action[x][y]]
            x = x2
            y = y2

        for i in range(len(path)):
            print path[i]

search(grid,init,goal,cost)

#-------------------------------
#A* Algorithm

# -----------
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
# 
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost,heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    f = 0

    open = [[f, g, x, y]] #first element is the f-value. g + h(x,y)

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            open.sort()
            open.reverse()
            next = open.pop() #choose node with smallest f-value
            x = next[2]
            y = next[3]
            g = next[1]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            f2 = g2 + heuristic[x2][y2] 
                            open.append([f2,g2, x2, y2])
                            closed[x2][y2] = 1

    return expand

astar = search(grid,init,goal,cost,heuristic)

for i in range(len(grid)):
    print astar[i]









