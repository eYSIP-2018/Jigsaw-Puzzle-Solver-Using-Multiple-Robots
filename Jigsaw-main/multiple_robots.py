import queue
import time
from itertools import combinations
from heapq import *
import math

import cv2
import cv2.aruco as aruco
import numpy as np
import serial
from xbee import XBee
import imutils

import aruco_lib
from communication import XBeeComm, Packets
from dijkstra import *
import pid_gui

from config import *


class Robot(XBeeComm, Packets, pid_gui.PID):
    """A class to create multiple instances for different robots to hold the state and position of the robot

    Attributes:
        ADDR (str): holds the 16 bit xbee xbee on robot
        MARKER_ID (int): id of the aruco on the robot
        destination ((int, int)): holds the destination where the robot needs to go
        center ((int, int)): holds the position of robot
        theta (int): holds the error angle by with the robot needs to turn to align to destination (pid input for robot)
        pid (pid_gui.PID class): a gui to tune the pid of the robot
        packet (Packets class): to create data packets to share with the robot

    Methods:
        update_position(): updates the position of the robot (i.e center, head, theta)
        get_position(): returns the the position of robot
        set_destination(): sets the destination of the robot
        send_packets(): sends packets to the robot
        align_robots(): aligns the the robot to the given point to make it face it in the direction of the point
        align_forward(): to maintain the robot at a particular distance from the obstacle
        pick_block(): signals the robot to pick the block and rotate it by the specified angle
        drop_block(): signals the robot to drop the block and rotate it by the specified angle
    """

    def __init__(self, addr, marker_id):
        """Creates an instance of Robot class
        Arguments:
            addr (str): 16 bit of the robots xbee
            marker_id (int): id of aruco on the robot
        """

        self.ADDR = addr
        self.MARKER_ID = marker_id

        self.destination = (0, 0)
        self.center, self.head, self.theta = ((0,0),(0,0),0)

        self.pid = pid_gui.PID('ROBOT_'+str(marker_id))

        self.packet = Packets()

        self.needs_align = False
        self.needs_to_move_forward = False
        self.target_peice_num = None
        self.target_peice_object = None
        self.target_peice_position = None
        self.target_peice_rotation = None
        self.reached = True
        self.selected_picking_destination = False
        self.reached_picking_destination = False
        self.selected_dropping_destination = False
        self.reached_dropping_destination = False

        self.needs_to_pick = False
        self.needs_to_drop = False

        self.stop_bit = 0
        self.path = []
        self.collision = False

        self.orientation_side = None

    def update_position(self):
        """Updates the postion of the robot"""

        try:
            self.center, self.head, self.theta = aruco_lib.get_marker_angle(frame, self.destination, self.MARKER_ID)
        except Exception as e:
            #print(e)
            self.theta = 0

    def get_position(self, mode="center"):
        """Returns the position of the robot
        Arguments:
            mode (str): "center" returns the cordinates of center of the robot, "head" returns the cordinates of head of the robot
        Retruns:
            ((int, int)): position of robot
        """
        if mode == "center":
            return self.center
        if mode == "head":
            return self.head

    def set_destination(self, pt):
        """Sets the destination of robot
        Arguments:
            pt ((int, int)): cordinates of the destination
        """
        self.destination = pt
        self.reached = False

    def get_distance(self, pt, mode="center"):
        """Returns the distance of robot from given point
        Arguments:
            pt ((int, int)): cordinates of the points
        Returns:
            (float): distance from a point
        """
        if mode == "center":
            return distance(self.center, pt)
        
        if mode == "head":
            return distance(self.head, pt)

    def send_packet(self, data):
        """Sends data packets to robot
        Arguments:
            data (tuple): a tuple containing the data to be sent to the robot
        """
        self.packet.resetData()
        self.packet.push(data)
        packet_data = self.packet.createPacket()
        comm.tx(packet_data, self.ADDR)

    def align_robot(self, pt):
        """Aligns the robot to face towards the given point
        Arguments:
            pt ((int, int)): cordinates of the point
        Returns:
            (bool): returns True if the robot is aligned else returns False
        """
        self.update_position()
        try:
            arucolength = distance(self.center, self.head)
            center_dist = distance(self.center, pt)
            head_dist = distance(self.head, pt)

            m2 = (self.center[1]-self.head[1])/(self.center[0]-self.head[0] + 0.001)
            m1 = (self.center[1]-pt[1])/(self.center[0]-pt[0] + 0.001)

            err_angle = np.arctan((m2-m1)/(1+m1*m2)+0.001)*180/np.pi

            if head_dist > np.sqrt(arucolength**2 + center_dist**2):
                if err_angle < 0:
                    err_angle = 180 + err_angle
                elif err_angle > 0:
                    err_angle = -180 + err_angle
                else:
                    err_angle = 180

            if err_angle > 10:
                self.send_packet(('Z', 1))
                return False
            elif err_angle < -10:
                self.send_packet(('Z', 2))
                return False
            else:
                self.send_packet(('Z', 0))
                return True

        except Exception as e:
            print(e)
            return False

        return False

    def align_forward(self):
        """Use this method to maintain a specified distance from the obstacle"""
        dist = self.get_distance(self.target_peice_position)
        if dist > DISTANCE_FROM_BLOCK + 5:
            self.send_packet(('F', 1))
            return False
        elif dist < DISTANCE_FROM_BLOCK - 5:
            self.send_packet(('F', 2))
            return False
        else:
            self.send_packet(('F', 0))
            return True
        
    def pick_block(self, angle):
        """Signals the Robot to pick up the block"""
        self.send_packet(('B', 1, angle))
        self.picked_block = True

    def drop_block(self, angle=0):
        """Signals the Robot to drop the block"""
        self.send_packet(('B', 2, angle))
        self.picked_block = False

    def move_towards_destination(self):
        """Sends data packets containing the error angle for pid and the pid tunning parameters"""
        global frame
        global blocks
        global robots

        self.stop_bit = 0

        if self.collision:
            self.stop_bit = 1

        self.blocks_map = cv2.resize(blocks.copy(), downscale(ARENA_SIZE), interpolation= cv2.INTER_CUBIC)

        self.pid.set_parameters()
        self.update_position()
        
        ###################################################################

        other_robots = [i for _, i in robots.items()]
        other_robots.remove(self)
        for r in other_robots:
            r.update_position()
            posx, posy = downscale(r.get_position())
            top_l = (posx - ROBOT_SIZE[0], posy - ROBOT_SIZE[1])
            bottom_r = (posx + ROBOT_SIZE[0], posy + ROBOT_SIZE[1])
            cv2.circle(self.blocks_map, (posx, posy), ROBOT_SIZE[0], 126, thickness=-1) 
            #cv2.rectangle(self.blocks_map, top_l, bottom_r, 126, -1)
            #dist_btw_robos = self.get_distance(r.get_position())

        ##################################################################

        head_dist = self.get_distance(self.destination, "center")
        if head_dist <= 10:
            self.reached = True
            self.stop_bit = 1
        else:
            try:
                self.path = astar(self.blocks_map, downscale(self.center), downscale(self.destination))
                self.intermediate_y, self.intermediate_x = upscale(self.path[1])
            except:
                self.intermediate_x, self.intermediate_y = self.destination

            try:
                self.center, self.head, self.theta = aruco_lib.get_marker_angle(frame, (self.intermediate_x, self.intermediate_y), self.MARKER_ID)
            except Exception as e:
                #print(e)
                self.theta = 0

        if self.collision:
            self.stop_bit = 1

        # self.update_position()
        self.packet.resetData()
        self.packet.push(('P', self.pid.kp, self.pid.ki, self.pid.kd))
        self.packet.push(('A', self.theta+180))
        self.packet.push(('S', self.stop_bit))

        packet_data = self.packet.createPacket()

        comm.tx(packet_data, self.ADDR)

    def display(self):
        """Draws points and lines to indicate the position of robot"""
        global frame
        global block

        cv2.line(frame, self.destination, self.center, (255, 255, 255), 1)
        cv2.circle(frame, self.destination, 5, (0,0,255), thickness=-1)
        cv2.circle(frame, self.center, 5, (0,255,0), thickness=-1)
        try:
            cv2.circle(frame, (self.intermediate_x, self.intermediate_y), 10, (0,255,255), thickness=-1)
        except Exception as e:
            pass
            #print(e)
        cv2.circle(frame, self.head, 5, (0,255,0), thickness=-1)


def distance(a, b):
    """Returns the distance between two diferent points
    Returns:
        (int): distance between the two points
    """
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def nearest_point(pt, pts_list):
    """Of list of points returns the point nearest to the given point
    Arguments:
        pt ((int, int)): given point
        pts_list (list): list of points
    """
    dist = 1000000
    nearest = None
    for i in pts_list:
        if dist > distance(pt, i[:2]):
            dist = distance(pt, i[:2])
            nearest = i
    return nearest

def get_adjacents(pt):
    """Returns a list containing adjacent points around the given point
    Arguments:
        pt ((int, int)): cordinates of the point
    Returns:
        adjacent (list): a list containing adjacent points
    """
    global blocks
    adjacents = []
    adjacents.append((pt[0]+DISTANCE_FROM_BLOCK, pt[1], 270))
    adjacents.append((pt[0]-DISTANCE_FROM_BLOCK, pt[1], 90))
    adjacents.append((pt[0], pt[1]+DISTANCE_FROM_BLOCK, 0))
    adjacents.append((pt[0], pt[1]-DISTANCE_FROM_BLOCK, 180))
    
    for i in adjacents[:]:
        try:
            if blocks[i[1], i[0]] <127:
                adjacents.remove(i)
        except:
            adjacents.remove(i)

    return adjacents

def get_nearest_obstacle(obstacles, i):
    """Returns the obstacle nearest to the given point
    Arguments:
        obstacles (dict): a dictionary containing the position of the obstacles as keys and width, height of obstacles as values
    Returns:
        ((int, int), (int, int)): positon of neareast obstacle, nearest adjacent point of the obstacle
    """
    near_obstacle = nearest_point(i, obstacles.keys())
    return near_obstacle, nearest_point(i, get_adjacents(near_obstacle))

def heuristic(a, b):
    """Returns the distance between two diferent points
    Returns:
        (int): distance between the two points
    """
    return abs(b[0] - a[0])**2 + abs(b[1] - a[1])**2

def astar(array, start, goal):
    """Returns a list of points in the path to be followed by the robot
    Arguments:
        array (numpy.array): map of obstacles
        start ((int, int)): cordinates of start point
        goal ((int, int)): cordinates of goal point
    Returns:
        path (list): list of cordinates in the path of robot
    Raises:
        "path not found" exception when no path exists between the 2 points
    """

    start = start[::-1]
    goal = goal[::-1]

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]

            data.reverse()
            path = []

            
            for i in data:
                path.append(i)   
            return path

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] < 127 :
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    raise Exception("path not found")

def get_obstacles(image):
    """Returns a dictionary containing the position of the obstacles as keys and width, height of obstacles as values
    and a dictionary containg the roi of puzzle peices their positions as keys
    Arguments:
        image (numpy.array): digital image of the arena configuration
    Returns:
        obstales, blocks_roi (dict, dict): a dictionary containing the position of the obstacles as keys and (width, height) of obstacles as values, 
            a dictionary containg the roi of puzzle peices their positions as keys
    """
    ih, iw = image.shape[:2]
    image_copy = image.copy()

    image = cv2.resize(image, ARENA_SIZE, interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray[np.where(gray == 0)]= 255

    ret,threshold = cv2.threshold(gray,200,255,cv2.THRESH_BINARY_INV)
    _, contours, heiarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    cont = []

    blocks = mask = np.zeros(threshold.shape[:2], np.uint8)

    blocks_roi = {}

    for i, c in enumerate(contours[1:]):

        rect  = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        mask_copy = mask.copy()
        cv2.drawContours(mask_copy, [box], -1, (255,255,255), 3)
        cv2.floodFill(mask_copy, None, (0,0), 255)
        mask_inv = cv2.bitwise_not(mask_copy)
        blocks = cv2.add(blocks, mask_inv)

    _, contours, heiarchy = cv2.findContours(blocks, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    obstacles = {}

    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        obstacles.update({(int(x+w/2), int(y+h/2)): BLOCK_SIZE})
        #obstacles.update({(int(x+w/2), int(y+h/2)): (w, h)}) # for unknown block sizes
        bottom_r = remap((x+w, y+h), ARENA_SIZE, (iw,ih))
        top_l = remap((x, y), ARENA_SIZE, (iw,ih))
        blocks_roi.update({(int(x+w/2), int(y+h/2)): image_copy[top_l[1]:bottom_r[1], top_l[0]:bottom_r[0]]})

    return obstacles, blocks_roi

def get_obstacles_map(obstacles):
    """Creates a obstacle map (image with pixel values less than 127 at places where there are obstacles)
    Arguments:
        obstacles (dict): a dictionary containing the position of the obstacles as keys and (width, height) of obstacles as values
    Returns:
        blocks (numpy.array): an image containing the obstacles as black pixels with safety area gray pixels
    """

    global placed_pecies
    
    blocks = np.zeros(ARENA_SIZE[::-1], np.uint8)
    grid = get_grid(ARENA_SIZE)

    for i in obstacles.keys():
        cv2.circle(blocks, i, int(CIRCULAR_SAFETY_FACTOR*BLOCK_SIZE[0]), 129, -1)

        cv2.rectangle(blocks, (i[0]-int(obstacles[i][0]/4), i[1]-int(obstacles[i][1]/4)), (i[0]+int(obstacles[i][0]/4), i[1]+int(obstacles[i][1]/4)), 255, -1)

    for i in placed_pecies.keys():
        try:
            if not i == grid[5]:
                cv2.circle(blocks, i, int(CIRCULAR_SAFETY_FACTOR*BLOCK_SIZE[0]), 129, -1)

            else:
                cv2.rectangle(blocks, (int(i[0]-5.5*placed_pecies[i][0]/4), int(i[1]-5.5*placed_pecies[i][1]/4)),
                    (int(i[0]+5.5*placed_pecies[i][0]/4), int(i[1]+5.5*placed_pecies[i][1]/4)), 129, -1)

            cv2.rectangle(blocks, (i[0]-int(placed_pecies[i][0]/4), i[1]-int(placed_pecies[i][1]/4)), (i[0]+int(placed_pecies[i][0]/4), i[1]+int(placed_pecies[i][1]/4)), 255, -1)
        except Exception as e:
            print(e)

    return cv2.bitwise_not(blocks)

def get_frame():
    """Returns a uindistorted frame removing fish eye effect
    Returns:
        dst (numpy.array): undistorted image of the arena
    """

    mtx = np.array(np.mat("588.4525598886621, 0, 301.8008794717551; 0, 588.9763096391521, 242.617026416902; 0, 0, 1"))
    dist = np.array(np.mat("-0.4351555722591889, 0.2082765081608728, -0.006072767012672472, 0.008139871640987759, 0"))

    ret, frame = cap.read()

    h,  w = frame.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h))

    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    h, w = dst.shape[:2]
    dst = dst[int(h/7):int(h*6/7), int(w/7):int(w*6/7)]
    dst = cv2.resize(dst, ARENA_SIZE, interpolation= cv2.INTER_CUBIC)

    return dst

def remap(i, from_size, to_size):
    """Remaps the pixel cordinate from from_size image to to_size image
    Arguments:
        i ((int, int)): point to be remapped
        from_size ((int, int)): size of source image
        to_size ((int, int)): size of destination image
    Returns:
        ((int, int)): remapped point
    """
    return (int(i[0]*to_size[0]/from_size[0]), int(i[1]*to_size[1]/from_size[1]))

def downscale(i, scale=RESIZE_SCALE):
    """Downscales the given point by given scale
    Arguments:
        i ((int, int)): point to be downscaled
        scale ((int, int)): scale to which the point should be downscaled
    Returns:
        ((int, int)): downscaled point
    """
    return (int(i[0]*scale), int(i[1]*scale))

def upscale(i, scale=RESIZE_SCALE):
    """Downscales the given point by given scale
    Arguments:
        i ((int, int)): point to be upscaled
        scale ((int, int)): scale to which the point should be upscaled
    Returns:
        ((int, int)): upscaled point
    """
    return (int(i[0]/scale), int(i[1]/scale))

def get_grid(size, blocksize=BLOCK_SIZE[0], padding=20):
    """Returns a dictionary containing the keys as peice number with value as cordinates where the block needs to be placed
    Arguments:
        size ((int, int)): size of arena
        blocksize (int): size of block
        padding (int): padding between blocks
    Returns:
        grid (dict): a dictionary containing the keys as peice number with value as cordinates where the block needs to be placed
    """
    h, w = size
    totalsize = blocksize + padding

    grid = {
        5: (int(h/2), int(w/2)),
        8: (int(h/2), int(w/2 + totalsize)),
        2: (int(h/2), int(w/2 - totalsize)),
        4: (int(h/2 - totalsize), int(w/2)),
        6: (int(h/2 + totalsize), int(w/2)),
        1: (int(h/2 - totalsize), int(w/2 - totalsize)),
        3: (int(h/2 + totalsize), int(w/2 - totalsize)),
        7: (int(h/2 - totalsize), int(w/2 + totalsize)),
        9: (int(h/2 + totalsize), int(w/2 + totalsize)),
    }

    return grid

def get_matches(jig_sol, blocks_roi):
    """Returns dictionary containing peice position as keys and (peice number, rotation of block) as values
    Arguments:
        jig_sol (numpy.array): image of jigsaw solution
        blocks_roi (dict): dictionary containing position of blocks as keys and roi of the block as values
    Returns:
        match_data (dict): dictionary containing peice position as keys and (peice number, rotation of block) as values
    """

    match_data = {}
    height, width,= jig_sol.shape
    font = cv2.FONT_HERSHEY_SIMPLEX

    identity = {
        (1, 1): 1,
        (2, 1): 2,
        (3, 1): 3,
        (1, 2): 4,
        (2, 2): 5,
        (3, 2): 6,
        (1, 3): 7,
        (2, 3): 8,
        (3, 3): 9,
    }

    for i in blocks_roi.keys():
        blk = blocks_roi[i].copy()
        blk = cv2.cvtColor(blk,cv2.COLOR_BGR2GRAY)
        max_list = []
        for k in range(0,360,90):
            #blk = np.rot90(blk)
            blk_copy = imutils.rotate_bound(blk, -k)
            #cv2.resize(blk,(int(width/3),int(height/3)), interpolation= cv2.INTER_CUBIC)
            result = cv2.matchTemplate(jig_sol,blk_copy,cv2.TM_CCOEFF_NORMED)

            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            max_list.append((max_loc, max_val, k))#((k+1)*90)%360))

        top_left = max(max_list, key=lambda x: x[1])[0]
        rot = max(max_list, key=lambda x: x[1])[2]
        bottom_right = (top_left[0] + int(width/3), top_left[1] + int(height/3))
        centx = 0
        centy = 0
        for (l,m) in [top_left, bottom_right]:
            centx += l
            centy += m

        centx = int(centx/2)
        centy = int(centy/2)
        piece = (math.ceil(3*centx/width), math.ceil(3*centy/height))

        if piece not in identity.keys():
            continue

        match_data.update({i: (identity[piece], rot)})

    return match_data

def search_by_peice_num(match_data, peice_num):
    """Returns the position of given peice number
    Arguments:
        match_data (dict): a dictionary containing keys as position of peices with values as peice number
        peice_num (int): query peice number
    Returns:
        i ((int, int)): peice position
    """
    for i in match_data.keys():
        num, _ = match_data[i]
        if num == peice_num:
            return i


global frame
global blocks

global placed_pecies
global robots

FIRST_PIECE = True

comm = XBeeComm()

#create instances of robots in a dictionary with their marker ids as keys
robots = {
    ROBOT_1_MARKER_ID: Robot(ROBOT_1_XBee_ADDR, ROBOT_1_MARKER_ID),
    ROBOT_2_MARKER_ID: Robot(ROBOT_2_XBee_ADDR, ROBOT_2_MARKER_ID),
}

DROPPING_LOCK = False

#initialize video capture
cap = cv2.VideoCapture("/dev/video0")

#initialize a dictionary to keep track of placed peices
placed_pecies = {}

#initialize a dictionary to keep track of peices that are not yet been taken by any other robots
available_peices = {}

#read the image containing the arena configuration
arena = cv2.imread('arena1.png', cv2.IMREAD_UNCHANGED)

if len(arena.shape) > 2 and arena.shape[2] == 4:
    arena = cv2.cvtColor(arena, cv2.COLOR_BGRA2BGR)

#get the obstacles and the images of their roi
obstacles, blocks_roi = get_obstacles(arena.copy())

#make all obstacle as available peices
available_peices.update(obstacles)

#load the solution image of the jigsaw puzzle
jigsaw_sol = cv2.imread('jigsaw_3.jpg')
jigsaw_sol = cv2.cvtColor(jigsaw_sol, cv2.COLOR_BGR2GRAY)

#solve the jigsaw puzzle and get the position of each peice
match_data = get_matches(jigsaw_sol, blocks_roi)

while True:

    #get the image of arena
    frame = get_frame()

    #create the obstacle map
    blocks = get_obstacles_map(obstacles)
    blocks_frame = blocks.copy()
    blocks_frame = cv2.resize(blocks_frame, ARENA_SIZE, interpolation= cv2.INTER_CUBIC)
    h, w = blocks.shape

    #get the dictionary containing the keys as peice number with value as cordinates where the block needs to be placed
    grid = get_grid(ARENA_SIZE)

    #find the robot nearest to the peice number 5 and make it pick the peice number 5
    if FIRST_PIECE:
        PEICE_NUM = 5

        #find the position of the peice
        DEST_KEY = search_by_peice_num(match_data, PEICE_NUM)

        #find the robot nearest to that peice
        dist = 1000
        for robo_key, robo in robots.items():
            robo.update_position()
            if dist > robo.get_distance(DEST_KEY):
                dist = robo.get_distance(DEST_KEY)
                first_robot = robo

        #find the nearest adjacent position
        dest = nearest_point(first_robot.get_position(), get_adjacents(DEST_KEY))
        dest, PICKUP_ORIENTATION = dest[:2], dest[2]

        first_robot.set_destination(dest)
        first_robot.target_peice_position = DEST_KEY
        available_peices.pop(first_robot.target_peice_position, None)
        first_robot.target_peice_num , first_robot.target_peice_rotation = match_data[first_robot.target_peice_position]
        first_robot.pickup_orientation = PICKUP_ORIENTATION
        first_robot.selected_picking_destination = True
        print("\n\n_________________________________________________________________")
        print("Going to pick peice no {} already rotated by an angle of {}\n".format(first_robot.target_peice_num, first_robot.target_peice_rotation))
        FIRST_PIECE = False


    #iterate for all robots
    for robo_key, robo in robots.items():

        #update the position of robot
        robo.update_position()

        if robo.collision:
            robo.send_packet(('S',1))
            continue
        else:
            robo.send_packet(('S',0))

        #align the robot to the target peice
        if robo.needs_align:
            if robo.align_robot(robo.target_peice_position):
                robo.needs_align = False
                robo.needs_to_move_forward = True
            else:
                cv2.circle(frame, robo.target_peice_position, 8, (0,255,255), thickness=-1)                
                continue

        #move the robot to maintain a particular destance from dropping or picking destination
        if robo.needs_to_move_forward:
            if robo.align_forward():
                robo.needs_to_move_forward = False
                robo.send_packet(('F', 0))
            continue

        #select the robots picking destination
        if not robo.selected_picking_destination:
            if robo.reached:
                robo.update_position()
                
                robo.target_peice_position, dest = get_nearest_obstacle(available_peices, robo.get_position())
                
                #remove the selected peice from the available peices
                available_peices.pop(robo.target_peice_position, None)
                dest, robo.pickup_orientation = dest[:2], dest[2]
                robo.set_destination(dest)
                robo.target_peice_num , robo.target_peice_rotation = match_data[robo.target_peice_position]
                print("\n\n_________________________________________________________________")
                print("Going to pick peice n.o {} already rotated by an angle of {} at {}\n".format(robo.target_peice_num, robo.target_peice_rotation, robo.target_peice_position))
                robo.selected_picking_destination = True

        if not robo.reached_picking_destination:
            robo.move_towards_destination()
            if robo.reached:
                robo.reached_picking_destination = True
                robo.needs_align = True
                robo.orientation_side = robo.pickup_orientation
                robo.needs_to_pick = True
            continue
        
        if robo.needs_to_pick:
            robo.update_position()
            robo.dropping_orientation = nearest_point(robo.get_position(), get_adjacents(grid[robo.target_peice_num]))[2]
            robo.pick_block((robo.pickup_orientation - robo.dropping_orientation - robo.target_peice_rotation +720)%360)
            robo.target_peice_object = obstacles.pop(robo.target_peice_position, None)
            robo.needs_to_pick = False
            print("Picked peice n.o {} and rotated it by {}".format(robo.target_peice_num, (robo.pickup_orientation - robo.dropping_orientation - robo.target_peice_rotation +720)%360))

        if not robo.selected_dropping_destination:
            if robo.reached:
                grid = get_grid(ARENA_SIZE)
                robo.update_position()
                dest = nearest_point(robo.get_position(), get_adjacents(grid[robo.target_peice_num]))
                dest, robo.dropping_orientation = dest[:2], dest[2]
                robo.set_destination(dest)
                robo.target_peice_position = grid[robo.target_peice_num]
                robo.selected_dropping_destination = True

        if not robo.reached_dropping_destination:
            robo.move_towards_destination()
            if robo.reached:
                robo.reached_dropping_destination = True
                robo.needs_align = True
                robo.orientation_side = robo.dropping_orientation
                robo.needs_to_drop = True
            continue
            
        if robo.needs_to_drop:
            robo.drop_block()
            placed_pecies.update({grid[robo.target_peice_num]: robo.target_peice_object})
            robo.needs_to_drop = False
            print("Dropped peice n.o {} and at {}".format(robo.target_peice_num, robo.target_peice_position))
            
        robo.needs_align = False
        robo.selected_picking_destination = False
        robo.reached_picking_destination = False
        robo.selected_dropping_destination = False
        robo.reached_dropping_destination = False

    ###################################################################################################

    reached = False
    for i in robots.keys():
        if robots[i].reached:
            reached = True
    
    if robots[ROBOT_1_MARKER_ID].get_distance(robots[ROBOT_2_MARKER_ID].get_position(mode="head"))<COLLISION_DISTANCE and not reached:
        if len(robots[ROBOT_1_MARKER_ID].path) > len(robots[ROBOT_2_MARKER_ID].path):
            robots[ROBOT_1_MARKER_ID].collision = True
            robots[ROBOT_2_MARKER_ID].collision = False
        else:
            robots[ROBOT_1_MARKER_ID].collision = False
            robots[ROBOT_2_MARKER_ID].collision = True
    else:
        robots[ROBOT_1_MARKER_ID].collision = False
        robots[ROBOT_2_MARKER_ID].collision = False

    #####################################################################################################

    for robo_key, robo in robots.items():
        try:
            robo.display()
            for i in robo.path:
                cv2.circle(frame, upscale((i[1], i[0])), 5, 0, thickness=-1)

            cv2.imshow(str(robo.MARKER_ID)+"-MAP", robo.blocks_map)
        except:
            pass

    for i in obstacles.keys():
        for j in get_adjacents(i):
            cv2.circle(frame, j[:2], 5, (0,255,0), thickness=-1)

    cv2.circle(frame, dest, 5, (255,255,0), thickness=-1)

    for i in grid.keys():
        cv2.circle(frame, grid[i], 5, (255, 0, 255), -1)

    '''
    for robo_key, robo in robots.items():
        posx, posy = robo.get_position(mode="head")
        cv2.circle(blocks, (posx, posy), int(ROBOT_SIZE[0]/RESIZE_SCALE), 129, thickness=-1)
        cv2.circle(frame, (posx, posy), COLLISION_DISTANCE, (0,255,0), thickness=1)
        '''
    
    blocks_frame = cv2.cvtColor(blocks,cv2.COLOR_GRAY2BGR)
    frame = cv2.bitwise_not(cv2.addWeighted(cv2.bitwise_not(frame),1, cv2.bitwise_not(blocks_frame), 0.7, 0))
    cv2.imshow('image',frame)
    cv2.imshow('blocks',blocks_frame)

    opt = cv2.waitKey(1) & 0xFF

    if opt == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
