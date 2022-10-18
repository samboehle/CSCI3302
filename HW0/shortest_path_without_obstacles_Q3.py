def shortest_path_grid(grid, start, goal):
    '''
    Function that returns the length of the shortest path in a grid
    that has no obstacles. The length is simply the number of cells
    on the path including the 'start' and the 'goal'

    :param grid: list of lists (represents a square grid)
    :param start: tuple of start index
    :param goal: tuple of goal index
    :return: length of path
    '''
    n = len(grid)
    #this just suptracts the values from the start and end position to get the ammount of cells the rover needs to move
    down = start[0] - goal[0]
    if(down < 0):
        down = down * -1
    
    cross = start[1] - goal[1]
    if(cross < 0):
        cross = cross * -1
    return cross + down +1


if __name__ == "__main__":
    grid = [[0,0,0],
            [0,0,0],
            [0,0,0]]
    start, goal = (0,0), (2,1)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 4

