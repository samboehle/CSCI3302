from tracemalloc import start
import numpy as np
import matplotlib.pyplot as plt
import math
import random

###############################################################################
## Base Code
###############################################################################




class Node:
    """
    Node for RRT Algorithm. This is what you'll make your graph with!
    """
    def __init__(self, pt, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)

def get_nd_obstacle(state_bounds):
    '''
    Function to return a circular obstacle in an n-dimensional world
    :param state_bounds: Array of min/max for each dimension
    :return: A single circular obstacle in form of a list with 1st entry as the circle center and the 2nd as the radius
    '''
    center_vector = []
    for d in range(state_bounds.shape[0]):
        center_vector.append(state_bounds[d][0] + random.random()*(state_bounds[d][1]-state_bounds[d][0]))
    radius = random.random() * 0.6 # Downscaling the radius
    return [np.array(center_vector), radius]

def setup_random_2d_world():
    '''
    Function that sets a 2D world with fixed bounds and # of obstacles
    :return: The bounds, the obstacles, the state_is_valid() function
    '''
    state_bounds = np.array([[0,10],[0,10]]) # matrix of min/max values for each dimension
    obstacles = [] # [pt, radius] circular obstacles
    for n in range(30):
        obstacles.append(get_nd_obstacle(state_bounds))

    def state_is_valid(state):
        '''
        Function that takes an n-dimensional point and checks if it is within the bounds and not inside the obstacle
        :param state: n-Dimensional point
        :return: Boolean whose value depends on whether the state/point is valid or not
        '''
        for dim in range(state_bounds.shape[0]):
            if state[dim] < state_bounds[dim][0]: return False
            if state[dim] >= state_bounds[dim][1]: return False
        for obs in obstacles:
            if np.linalg.norm(state - obs[0]) <= obs[1]: return False
        return True

    return state_bounds, obstacles, state_is_valid

def setup_fixed_test_2d_world():
    '''
    Function that sets a test 2D world with fixed bounds and # of obstacles
    :return: The bounds, the obstacles, the state_is_valid() function
    '''
    state_bounds = np.array([[0,1],[0,1]]) # matrix of min/max values for each dimension
    obstacles = [] # [pt, radius] circular obstacles
    obstacles.append([[0.5,0.5],0.2])
    obstacles.append([[0.1,0.7],0.1])
    obstacles.append([[0.7,0.2],0.1])

    # Pretty wild but you can have nested functions in python and in this case it will retain
    # its local variables state_bounds and obstacles. You won't need to pass them later.
    def state_is_valid(state):
        '''
        Function that takes an n-dimensional point and checks if it is within the bounds and not inside the obstacle
        :param state: n-Dimensional point
        :return: Boolean whose value depends on whether the state/point is valid or not
        '''
        for dim in range(state_bounds.shape[0]):
            if state[dim] < state_bounds[dim][0]: return False
            if state[dim] >= state_bounds[dim][1]: return False
        for obs in obstacles:
            if np.linalg.norm(state - obs[0]) <= obs[1]: return False
        return True

    return state_bounds, obstacles, state_is_valid

def _plot_circle(x, y, radius, color="-k"):
    '''
    Internal function to plot a 2D circle on the current pyplot object
    :param x: The x coordinate of the circle
    :param y: The y coordinate of the circle
    :param radius: The radius of the circle
    :param color: Matplotlib color code
    :return: None
    '''
    deg = np.linspace(0,360,50)

    xl = [x + radius * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + radius * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)

def visualize_2D_graph(state_bounds, obstacles, nodes, goal_point=None, filename=None):
    '''
    Function to visualise the 2D world, the RRT graph, path to goal if goal exists
    :param state_bounds: Array of min/max for each dimension
    :param obstacles: Locations and radii of spheroid obstacles
    :param nodes: List of vertex locations
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param filename: Complete path to the file on which this plot will be saved
    :return: None
    '''
    fig = plt.figure()
    plt.xlim(state_bounds[0,0], state_bounds[0,1])
    plt.ylim(state_bounds[1,0], state_bounds[1,1])

    # for obs in obstacles:
    #_plot_circle(obs[0][0], obs[0][1], obs[1])

    goal_node = None
    for node in nodes:
        if node.parent is not None:
            node_path = np.array(node.path_from_parent)
            plt.plot(node_path[:,0], node_path[:,1], '-b')
        # The goal may not be on the RRT so we are finding the point that is a 'proxy' for the goal
        if goal_point is not None and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5:
            goal_node = node
            plt.plot(node.point[0], node.point[1], 'k^')
        else:
            plt.plot(node.point[0], node.point[1], 'ro')

    plt.plot(nodes[0].point[0], nodes[0].point[1], 'ko')

    if goal_node is not None:
        cur_node = goal_node
        while cur_node is not None: 
            if cur_node.parent is not None:
                node_path = np.array(cur_node.path_from_parent)
                plt.plot(node_path[:,0], node_path[:,1], '-y')
                cur_node = cur_node.parent
            else:
                break

    if goal_point is not None:
        plt.plot(goal_point[0], goal_point[1], 'gx')

    if filename is not None:
        fig.savefig(filename)
    else:
        plt.show()

def get_random_valid_vertex(state_is_valid, bounds):
    '''
    Function that samples a random n-dimensional point which is valid (i.e. collision free and within the bounds)
    :param state_valid: The state validity function that returns a boolean
    :param bounds: The world bounds to sample points from
    :return: n-Dimensional point/state
    '''
    vertex = None
    while vertex is None: # Get starting vertex
        pt = np.random.rand(bounds.shape[0]) * (bounds[:,1]-bounds[:,0]) + bounds[:,0]
        if state_is_valid(pt):
            vertex = pt
    return vertex

###############################################################################
## END BASE CODE
###############################################################################

def get_nearest_vertex(node_list, q_point):
    '''
    Function that finds a node in node_list with closest node.point to query q_point
    :param node_list: List of Node objects
    :param q_point: n-dimensional array representing a point
    :return Node in node_list with closest node.point to query q_point
    '''

    close_dist = np.linalg.norm(node_list[0].point - q_point)
    ret = node_list[0]

    for x in node_list:
        cur = x
        if np.linalg.norm(cur.point -q_point) < close_dist:
            close_dist = np.linalg.norm(cur.point -q_point)
            ret = cur
    
    return ret



def steer(from_point, to_point, delta_q):
    '''
    :param from_point: n-Dimensional array (point) where the path to "to_point" is originating from (e.g., [1.,2.])
    :param to_point: n-Dimensional array (point) indicating destination (e.g., [0., 0.])
    :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
    :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
    '''
    # TODO: Figure out if you can use "to_point" as-is, or if you need to move it so that it's only delta_q distance away

    # TODO Use the np.linspace function to get 10 points along the path from "from_point" to "to_point"
    displacement = to_point - from_point
    distance = np.linalg.norm(displacement)

    if(distance > delta_q):
        to_point = from_point + (delta_q * (displacement/distance))

    return np.linspace(from_point, to_point, num =10)



def check_path_valid(path, state_is_valid):
    '''
    Function that checks if a path (or edge that is made up of waypoints) is collision free or not
    :param path: A 1D array containing a few (10 in our case) n-dimensional points along an edge
    :param state_is_valid: Function that takes an n-dimensional point and checks if it is valid
    :return: Boolean based on whether the path is collision free or not
    '''

    for x in path:
        if state_is_valid(x) == False:
            return False
        
    return True

def rrt(state_bounds, state_is_valid, starting_point, goal_point, k, delta_q):
    '''
    TODO: Implement the RRT algorithm here, making use of the provided state_is_valid function.
    RRT algorithm.
    If goal_point is set, your implementation should return once a path to the goal has been found 
    (e.g., if q_new.point is within 1e-5 distance of goal_point), using k as an upper-bound for iterations. 
    If goal_point is None, it should build a graph without a goal and terminate after k iterations.

    :param state_bounds: matrix of min/max values for each dimension (e.g., [[0,1],[0,1]] for a 2D 1m by 1m square)
    :param state_is_valid: function that maps states (N-dimensional Real vectors) to a Boolean (indicating free vs. forbidden space)
    :param starting_point: Point within state_bounds to grow the RRT from
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param k: Number of points to sample
    :param delta_q: Maximum distance allowed between vertices
    :returns List of RRT graph nodes
    '''
    node_list = []
    node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent

    # TODO: Your code here
    # TODO: Make sure to add every node you create onto node_list, and to set node.parent and node.path_from_parent for each
    for x in range(0, k):

        #source to wikipedia and lecture slides for psuedocode for algorithim: https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree

        #get a random point in the map that is valid
        q_rand = get_random_valid_vertex(state_is_valid, state_bounds)

        #if we have goal point then sometimes set q_rand to goal point to move tree in that direction
        if(goal_point is not None):
            if(random.random() < .05):
                q_rand = goal_point
        
        #find the closest node that we already have in the list
        q_near = get_nearest_vertex(node_list, q_rand)

        #find a path from our old node in the list to are new new
        q_new = steer(q_near.point, q_rand, delta_q)

        #checking to see if the path goes over any obstacles
        if check_path_valid(q_new, state_is_valid):
            nn = Node(q_new[-1], parent= q_near)
            nn.path_from_parent = q_new
            node_list.append(nn)

            if goal_point is not None:
                if np.linalg.norm((nn.point - goal_point) / 10) < .00001:
                    return node_list



    return node_list

# def valid(state):
#     # obstacles = []
#     # obstacles.append([0,1],)
#     return True

# if __name__ == "__main__":
#     K = 250 # Feel free to adjust as desired
#     nodes = rrt(np.array([[0,10],[0,10]]), valid, np.array([0,0]), np.array([0,3]), 100, .5)
#     visualize_2D_graph(np.array([[0,10],[0,10]]), None, nodes, np.array([0,3]), 'rrt_run0.png')


    # bounds, obstacles, validity_check = setup_fixed_test_2d_world()
    # starting_point = get_random_valid_vertex(validity_check, bounds)
    # nodes = rrt(bounds, validity_check, starting_point, None, K, np.linalg.norm(bounds/10.))
    # visualize_2D_graph(bounds, obstacles, nodes, None, 'rrt_run1.png')

    # bounds, obstacles, validity_check = setup_random_2d_world()
    # starting_point = get_random_valid_vertex(validity_check, bounds)
    # nodes = rrt(bounds, validity_check, starting_point, None, K, np.linalg.norm(bounds/10.))
    # visualize_2D_graph(bounds, obstacles, nodes, None, 'rrt_run2.png')

    # bounds, obstacles, validity_check = setup_fixed_test_2d_world()
    # starting_point = get_random_valid_vertex(validity_check, bounds)
    # goal_point = get_random_valid_vertex(validity_check, bounds)
    # while np.linalg.norm(starting_point - goal_point) < np.linalg.norm(bounds/2.):
    #     starting_point = get_random_valid_vertex(validity_check, bounds)
    #     goal_point = get_random_valid_vertex(validity_check, bounds)
    # nodes = rrt(bounds, validity_check, starting_point, goal_point, K, np.linalg.norm(bounds/10.))
    # visualize_2D_graph(bounds, obstacles, nodes, goal_point, 'rrt_goal_run1.png')

    # bounds, obstacles, validity_check = setup_random_2d_world()
    # starting_point = get_random_valid_vertex(validity_check, bounds)
    # goal_point = get_random_valid_vertex(validity_check, bounds)
    # while np.linalg.norm(starting_point - goal_point) < np.linalg.norm(bounds/2.):
    #     starting_point = get_random_valid_vertex(validity_check, bounds)
    #     goal_point = get_random_valid_vertex(validity_check, bounds)
    # nodes = rrt(bounds, validity_check, starting_point, goal_point, K, np.linalg.norm(bounds/10.))
    # visualize_2D_graph(bounds, obstacles, nodes, goal_point, 'rrt_goal_run2.png')



#List of all coordinates that robot needs to go to append current position to front
