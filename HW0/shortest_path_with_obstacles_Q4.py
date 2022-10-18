def shortest_path_grid(grid, start, goal):
        '''
        Function that returns the length of the shortest path in a grid
        that HAS obstacles represented by 1s. The length is simply the number
        of cells on the path including the 'start' and the 'goal'

        :param grid: list of lists (represents a square grid where 0 represents free space and 1s obstacles)
        :param start: tuple of start index
        :param goal: tuple of goal index
        :return: length of path
        '''
        n = len(grid)

        #grid to store length of path for each spot
        length = [[0 for x in range(n)] for x in range(n)]

        #grid to store prev spot of each spot
        prev = [[[tuple()] for x in range(n)] for x in range(n)]

        #list to see if node has been visited
        visited = []

        #queue for bfs
        queue = []

        queue.append(start)


        prev[start[0]][start[1]] = (start[0], start[1])
        while(queue):
                current = queue.pop(0)
                visited.append(current)

                #checking to see if spot is a 1 
                if(grid[current[0]][current[1]] == 1):
                        continue
                prev0 = prev[current[0]][current[1]][0]
                prev1 = prev[current[0]][current[1]][1]
                length[current[0]][current[1]] = length[prev0][prev1] + 1

                if(current == goal):
                        return length[current[0]][current[1]]
                #move down
                if(current[0] +1 < n and current[0]+ 1 >= 0):
                        if(grid[current[0] +1][current[1]] != 1):
                                if (current[0] + 1, current[1]) not in visited:
                                        queue.append((current[0] + 1, current[1]))
                                        prev[current[0] + 1][current[1]] = (current[0], current[1])
                #move left
                if(current[1] -1 < n and current[1]- 1 >= 0):
                        if(grid[current[0]][current[1] -1] != 1):
                                if (current[0], current[1] -1) not in visited:
                                        queue.append((current[0], current[1] -1))               
                                        prev[current[0]][current[1] -1] = (current[0], current[1])
                #move up
                if(current[0] -1 < n and current[0]- 1 >= 0):
                        if(grid[current[0] -1][current[1]] != 1):
                                if (current[0] - 1, current[1]) not in visited:
                                        queue.append((current[0] - 1, current[1]))
                                        prev[current[0] - 1][current[1]] = (current[0], current[1])
                #move right
                if(current[1] +1 < n and current[1]+ 1 >= 0):
                        if(grid[current[0]][current[1] +1] != 1):
                                if (current[0], current[1] +1) not in visited:
                                        queue.append((current[0], current[1] +1))               
                                        prev[current[0]][current[1] +1] = (current[0], current[1])
        return -1








if __name__ == "__main__":
    grid = [[0,0,0],
            [1,1,0],
            [1,1,0]]
    start, goal = (0,1),(2,2)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 4

    grid = [[0,1],
            [1,0]]
    start, goal = (0, 0), (1,1)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == -1