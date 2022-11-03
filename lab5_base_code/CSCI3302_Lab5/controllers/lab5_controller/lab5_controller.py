"""lab5 controller."""
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


##### vvv [Begin] Do Not Modify vvv #####

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 12x12m2 apartment
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
##### ^^^ [End] Do Not Modify ^^^ #####

##################### IMPORTANT #####################
# Set the mode here. Please change to 'autonomous' before submission
#mode = 'manual' # Part 1.1: manual mode
mode = 'planner'
# mode = 'autonomous'




###################
#
# Planner
#
###################
if mode == 'planner':
    # Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    start_w = (2.44, 5.13) # (Pose_X, Pose_Z) in meters
    end_w = (10, 7) # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_w from the webots coordinate frame into the map frame
    start = (360-int(start_w[1] * 30), int(start_w[0]  *30)) # (x, y) in 360x360 map
    end = (360-int(end_w[1] * 30), int(end_w[0]  *30)) # (x, y) in 360x360 map
    
    print(start, end)
        
    def unroll(q):
        return [] if q[1] is None else unroll(q[1]) + [q[0]]
        
    def h(a, b):
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

    # Part 2.3: Implement A* or Dijkstra's Algorithm to find a path
    def path_planner(grid, start, goal):
        start = (start[1], start[0])
        goal = (goal[1], goal[0])

        queue = [(start, None, 0, h(start, goal))]
        visit = []
        iter = 0
        while len(queue) > 0 and iter < 1000:
            q = queue.pop(0)
            if q[0] == goal:
                return unroll(q)
            for r in range(-1, 2):
                for c in range(-1, 2):
                    if r == 0 and c == 0:
                        continue
                    i = (r + q[0][0], c + q[0][1])
                    if not (i[0] >= 0 and i[1] >= 0 and i[0] < len(grid) and i[1] < len(grid[0])) or grid[i[0]][i[1]]:
                        continue
                    cost = q[2] + h(q[0], i) + h(i, goal)
                    if i in visit:
                        bestSpot = len(queue)
                        for p in range(len(queue)):
                            if cost < queue[p][3]:
                                bestSpot = min(bestSpot, p)
                            if queue[p][0] == i:
                                if bestSpot != len(queue):
                                    queue.pop(p)
                                    queue.insert(bestSpot, (i, q, q[2] + h(q[0], i), cost))
                                break
                    else:
                        bestSpot = len(queue)
                        for p in range(len(queue)):
                            if cost < queue[p][3]:
                                bestSpot = p
                                break
                        queue.insert(bestSpot, (i, q, q[2] + h(q[0], i), cost))
                        visit.append(i)
        return []

    # Part 2.1: Load map (map.npy) from disk and visualize it
    m = np.load("map.npy")
    m = np.fliplr(np.rot90(m, k=3))
    c = np.ones((11,11))
    b = convolve2d(m,c*20)
    # b = np.min(1,b)
    bmap = b > 1
    imap = bmap * 1
    
    
    
    path = path_planner(imap, start, end)
   # plt.imshow(imap)
    for point in path:
        imap[point[0]][point[1]] = 2
    # plt.imshow(m)    
    plt.imshow(imap)
    plt.show()

    # Part 2.2: Compute an approximation of the “configuration space”
    # 156, 337
    # 310, 56

    

    # Part 2.3 continuation: Call path_planner


    # Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    np.save("path.npy", path)
    waypoints = []

######################
#
# Map Initialization
#
######################

# Part 1.2: Map Initialization

# Initialize your map data structure here as a 2D floating point array
map = np.zeros(shape=(360,360)) # Replace None by a numpy 2D floating point array
waypoints = []

if mode == 'autonomous':
    # Part 3.1: Load path from disk and visualize it
    waypoints = np.load("path.npy") # Replace with code to load your path

state = 0 # use this to iterate through your path
maxp = 0
stop = False


while robot.step(timestep) != -1 and mode != 'planner' and not stop:

    ###################
    #
    # Mapping
    #
    ###################

    ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    pose_y = gps.getValues()[2]
    pose_x = gps.getValues()[0]
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
    
        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.

            # You will eventually REPLACE the following 2 lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            ry = 360-int(wy*30)
            rx = int(wx*30)
            if ry < 0:
                ry = 0
            if ry >= 360:
                ry = 359
            if rx < 0:
                rx = 0
            if rx >= 360:
                rx = 359 
            map[ry][rx] += 0.005
            map[ry][rx] = min(map[ry][rx], 1)
            g = int(map[ry][rx] * 255)
            color = g*256**2+g*256+g
            display.setColor(color)
            display.drawPixel(ry,rx)

    # Draw the robot's current pose on the 360x360 display
    display.setColor(int(0xFF0000))
    display.drawPixel(360-int(pose_y*30),int(pose_x*30))



    ###################
    #
    # Controller
    #
    ###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: Filter map and save to filesystem
            bmap = map > 0.5
            imap = bmap * 1
            print(imap)
            np.save("map.npy", imap)
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = nps.load("map.npy")
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else: # not manual mode
        # Part 3.2: Feedback controller
        #STEP 1: Calculate the error
        def h(a, b):
            return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5
            
        def angleSub(a, b):
            c = math.fmod(a - b, math.pi * 2)
            if abs(c) < math.pi:
                return c
            if c > 0:
                return c - math.pi * 2
            return c + math.pi * 2
        
            
        
        g = (int(pose_x  *30), 360-int(pose_y * 30))
        
        
        

        for i in range(maxp, len(waypoints)):
            
            # print(h(g, waypoints[i]), g , waypoints[i])
            display.setColor(int(0xFFFFFF))
            display.drawPixel(int(waypoints[i][1]), int(waypoints[i][0]))
            if h(g, waypoints[i]) <= 10:
                maxp = i
                if i == len(waypoints)-1 and h(g, waypoints[i]) <= 3:
                    stop = True
        
        goal = waypoints[maxp]
        display.setColor(int(0x00FF00))
        for i in range(-1,2):
            for j in range(-1,2):
                display.drawPixel(int(goal[1]) + i, int(goal[0]) + j)
        p = h(g, goal)
        goal = ((goal[0]) / 30 ,(goal[1]- 360) / -30)
        
        alpha = angleSub(-math.atan2(goal[1]-pose_y,goal[0]-pose_x), pose_theta)
        
        ###########################
        
        
        # STEP 2.2: Feedback Controller
        dx = 1 if abs(alpha) < 0.3 else 0
        d_theta = alpha
        
        # STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
        # Note that vL and vR in code is phi_l and phi_r on the slides/lecture
    
        pl = (dx - (d_theta * AXLE_LENGTH/2))
        pr = (dx + (d_theta * AXLE_LENGTH/2))
        
        m = max(abs(pl), abs(pr))
        if m > MAX_SPEED_MS/2:
            m /= MAX_SPEED_MS/2
            pl /= m
            pr /= m
    
    
    
    
        # STEP 2.3: Proportional velocities
        vL = (pl/MAX_SPEED_MS) * MAX_SPEED # Left wheel velocity in rad/s
        vR = (pr/MAX_SPEED_MS) * MAX_SPEED # Right wheel velocity in rad/s
     
        # print("Current pose: [%5f, %5f, %5f], vL: %5f vR: %5f p: %5f a: %5f maxp: %5d" % (pose_x, pose_y, pose_theta,vL,vR, p, alpha, maxp))
        # STEP 2.4: Clamp wheel speeds
        if vL > MAX_SPEED:
            vL = MAX_SPEED
        elif vL < -MAX_SPEED:
            vL = -MAX_SPEED
        if vR > MAX_SPEED:
            vR = MAX_SPEED
        elif vR < -MAX_SPEED:
            vR = -MAX_SPEED
            
        if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
        if pose_theta < -3.14: pose_theta += 6.28
    
    ##################################

        # Normalize wheelspeed
        # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)


    # Odometry code. Don't change vL or vR speeds after this line.
    # We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    # print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    
robot_parts[MOTOR_LEFT].setVelocity(0)
robot_parts[MOTOR_RIGHT].setVelocity(0)