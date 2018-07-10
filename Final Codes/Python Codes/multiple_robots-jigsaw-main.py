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
import improc
import pid_gui
from path_planning import heuristic, astar

from config import *
from misc import *


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

        self.dropping = False
        self.blocked = False

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

        #push data to be sent as packets to packets list
        self.packet.push(data)

        #create packet using the the data in packets list
        data_packet = self.packet.createPacket()

        #send packet to the robot
        comm.tx(data_packet, self.ADDR)

    def align_robot(self, pt):
        """Aligns the robot to face towards the given point
        Arguments:
            pt ((int, int)): cordinates of the point
        Returns:
            (bool): returns True if the robot is aligned else returns False
        """

        #update the position of robot
        self.update_position()

        try:
            #find the length of aruco
            arucolength = distance(self.center, self.head)

            #find the distance of the point from aruco center
            center_dist = distance(self.center, pt)

            #find the distance of point from the midpoint of one of the edges of the aruco
            head_dist = distance(self.head, pt)

            #find the slope of aruco
            m2 = (self.center[1]-self.head[1])/(self.center[0]-self.head[0] + 0.001)

            #find the slope of line connecting the aruco center and the point
            m1 = (self.center[1]-pt[1])/(self.center[0]-pt[0] + 0.001)

            #find the angle between aruco and line joining the point and aruco center
            err_angle = np.arctan((m2-m1)/(1+m1*m2)+0.001)*180/np.pi

            #find if the angle is obtuse
            if head_dist > np.sqrt(arucolength**2 + center_dist**2):

                #add 180 if the angle negative
                if err_angle < 0:
                    err_angle = 180 + err_angle

                #substract 180 if the angle is positive
                elif err_angle > 0:
                    err_angle = -180 + err_angle
                
                #if angle is 0
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

        #find the distance of robot from the target peice
        dist = self.get_distance(self.target_peice_position)

        #if the distance of the robot is more than acceptable move the robot forward
        if dist > DISTANCE_FROM_BLOCK + 5:
            self.send_packet(('F', 1))
            return False

        #if the distance of the robot is less than acceptable move the robot backward
        elif dist < DISTANCE_FROM_BLOCK - 5:
            self.send_packet(('F', 2))
            return False

        #if the distance of the robot is acceptable do nothing
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

        #if collision set stop bit to 1
        if self.collision:
            self.stop_bit = 1

        #binary image consiting of obstacles with pixel values less than 127
        self.blocks_map = cv2.resize(blocks.copy(), downscale(ARENA_SIZE), interpolation= cv2.INTER_CUBIC)

        #get the pid tuinning values from the gui trackbar
        self.pid.set_parameters()
        self.update_position()
        
        
        #MULTIPLE ROBOTS CODE
        ################################################################################

        #find the robots other than self
        other_robots = [i for _, i in robots.items()]
        other_robots.remove(self)

        #update other robots as obstacles in the obstacles map only if the distance between the robots is less than 150
        for r in other_robots:
            r.update_position()
            posx, posy = downscale(r.get_position())
            dist_btw_robos = self.get_distance(r.get_position())
            if dist_btw_robos>150:
                continue
            cv2.circle(self.blocks_map, (posx, posy), ROBOT_SIZE[0], 126, thickness=-1) 
        ################################################################################
        
        #find the distance of destination from center
        head_dist = self.get_distance(self.destination, "center")

        #if the distance is less than 10 than stop the robot and set self.reached to true
        if head_dist <= 10:
            self.reached = True
            self.stop_bit = 1
        else:
            try:
                #find the path from robot center to destination
                self.path = astar(self.blocks_map, downscale(self.center), downscale(self.destination))

                #select the first element in the path list as the intermidiate destination
                self.intermediate_y, self.intermediate_x = upscale(self.path[1])
            except Exception as e:

                #if path not found and the destination near set the destination as intermidiate destination
                if head_dist<30:
                    self.intermediate_x, self.intermediate_y = self.destination

            try:
                self.center, self.head, self.theta = aruco_lib.get_marker_angle(frame, (self.intermediate_x, self.intermediate_y), self.MARKER_ID)
            except Exception as e:
                #print(e)
                self.theta = 0

        #empty packets list
        self.packet.resetData()

        #push the pid tunning values to the packets list
        self.packet.push(('P', self.pid.kp, self.pid.ki, self.pid.kd))

        #push the error angle to the packets list
        self.packet.push(('A', self.theta+180))

        #push the stop bit to the packets list
        self.packet.push(('S', self.stop_bit))

        #create the data packet
        data_packet = self.packet.createPacket()

        #send the data packet to the robot
        comm.tx(data_packet, self.ADDR)

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

global frame
global blocks

global placed_pecies
global robots
global count

FIRST_PIECE = True

puzzle_completed = False
FIVE_PLACED = False

comm = XBeeComm()

#create instances of robots in a dictionary with their marker ids as keys
robots = {
    ROBOT_1_MARKER_ID: Robot(ROBOT_1_XBee_ADDR, ROBOT_1_MARKER_ID),
    ROBOT_2_MARKER_ID: Robot(ROBOT_2_XBee_ADDR, ROBOT_2_MARKER_ID),
}

#initialize video capture
cap = cv2.VideoCapture(CAMERA_PORT)

#initialize a dictionary to keep track of placed peices
placed_pecies = {}

#initialize a dictionary to keep track of peices that are not yet been taken by any other robots
available_peices = {}

#read the image containing the arena configuration
arena = cv2.imread(ARENA_IMAGE, cv2.IMREAD_UNCHANGED)

if len(arena.shape) > 2 and arena.shape[2] == 4:
    arena = cv2.cvtColor(arena, cv2.COLOR_BGRA2BGR)

#get the obstacles and the images of their roi
obstacles, blocks_roi = improc.get_obstacles(arena.copy())

#make all obstacle as available peices
available_peices.update(obstacles)

#load the solution image of the jigsaw puzzle
jigsaw_sol = cv2.imread(JIGSAW_SOLUTION)
jigsaw_sol = cv2.cvtColor(jigsaw_sol, cv2.COLOR_BGR2GRAY)

#solve the jigsaw puzzle and get the position of each peice
match_data = improc.get_matches(jigsaw_sol, blocks_roi)

while True:
    #get the image of arena
    frame = improc.get_frame(cap)

    #create the obstacle map
    blocks = improc.get_obstacles_map(obstacles, placed_pecies)
    blocks_frame = blocks.copy()
    blocks_frame = cv2.resize(blocks_frame, ARENA_SIZE, interpolation= cv2.INTER_CUBIC)
    h, w = blocks.shape

    #get the dictionary containing the keys as peice number with value as cordinates where the block needs to be placed
    grid = get_grid(ARENA_SIZE)


    #MULTIPLE ROBOTS CODE
    ###################################################################################################
    reached = False
    for i in robots.keys():
        if robots[i].reached:
            reached = True
    
    if robots[ROBOT_1_MARKER_ID].get_distance(robots[ROBOT_2_MARKER_ID].get_position(mode="head"))<COLLISION_DISTANCE and not reached:
        if len(robots[ROBOT_1_MARKER_ID].path) < len(robots[ROBOT_2_MARKER_ID].path):
            robots[ROBOT_1_MARKER_ID].collision = True
            robots[ROBOT_2_MARKER_ID].collision = False
        else:
            robots[ROBOT_1_MARKER_ID].collision = False
            robots[ROBOT_2_MARKER_ID].collision = True
    else:
        robots[ROBOT_1_MARKER_ID].collision = False
        robots[ROBOT_2_MARKER_ID].collision = False

    '''
    if robots[ROBOT_1_MARKER_ID].blocked or robots[ROBOT_2_MARKER_ID].blocked:
        robots[ROBOT_1_MARKER_ID].collision = False
        robots[ROBOT_2_MARKER_ID].collision = False
    '''
    #####################################################################################################


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
        dest = nearest_point(first_robot.get_position(), get_adjacents(DEST_KEY, blocks))
        dest, PICKUP_ORIENTATION = dest[:2], dest[2]

        first_robot.set_destination(dest)
        first_robot.target_peice_position = DEST_KEY
        available_peices.pop(first_robot.target_peice_position, None)
        first_robot.target_peice_num , first_robot.target_peice_rotation = match_data[first_robot.target_peice_position]
        first_robot.pickup_orientation = PICKUP_ORIENTATION
        first_robot.selected_picking_destination = True
        #print("\n\n_________________________________________________________________")
        #print("Going to pick peice no {} already rotated by an angle of {}\n".format(first_robot.target_peice_num, first_robot.target_peice_rotation))
        FIRST_PIECE = False

    #iterate for all robots
    for robo_key, robo in robots.items():

        if len(list(available_peices.keys())) == 0:
            puzzle_completed = True

        if not FIVE_PLACED and robo is not first_robot:
            continue

        #update the position of robot
        robo.update_position()


        #MULTIPLE ROBOTS CODE
        ###################################################
        other_robot = [i for _, i in robots.items()]
        other_robot.remove(robo)
        other_robot = other_robot[0]

        if robo.collision:
            robo.send_packet(('S',1))
            continue
        ###################################################


        #align the robot to the target peice
        if robo.needs_align:
            if robo.align_robot(robo.target_peice_position):
                robo.needs_align = False
                robo.needs_to_move_forward = True
            else:
                cv2.circle(frame, robo.target_peice_position, 8, (0,255,255), thickness=-1)                
                continue

        #move the robot to maintain a particular distance from dropping or picking destination
        if robo.needs_to_move_forward:
            if robo.align_forward():
                robo.needs_to_move_forward = False
                robo.send_packet(('F', 0))
            continue

        #select the robots picking destination
        if not robo.selected_picking_destination:
            if robo.reached:
                robo.update_position()

                #dont pick a new destination if no new peices are available
                if len(list(available_peices.keys())) == 0:
                    continue
                                    
                robo.target_peice_position, dest = get_nearest_obstacle(blocks, available_peices, robo.get_position())
                
                #remove the selected peice from the available peices
                available_peices.pop(robo.target_peice_position, None)
                dest, robo.pickup_orientation = dest[:2], dest[2]
                robo.set_destination(dest)
                robo.target_peice_num , robo.target_peice_rotation = match_data[robo.target_peice_position]
                #print("\n\n_________________________________________________________________")
                #print("Going to pick peice n.o {} already rotated by an angle of {} at {}\n".format(robo.target_peice_num, robo.target_peice_rotation, robo.target_peice_position))
                robo.selected_picking_destination = True

        #make the robot move to towards the destination
        if not robo.reached_picking_destination:
            robo.move_towards_destination()
            if robo.reached:
                robo.reached_picking_destination = True
                robo.needs_align = True
                robo.needs_to_pick = True
            if blocks[robo.destination[::-1]] < 127:
                robo.update_position()
                dest = nearest_point(robo.get_position(), get_adjacents(robo.target_peice_position, blocks))
                dest, robo.dropping_orientation = dest[:2], dest[2]
                robo.set_destination(dest)
            continue

        if robo.needs_to_pick:
            robo.update_position()
            robo.dropping_orientation = nearest_point(robo.get_position(), get_adjacents(grid[robo.target_peice_num], blocks))[2]
            robo.pick_block((robo.pickup_orientation - robo.dropping_orientation - robo.target_peice_rotation +720)%360)
            robo.target_peice_object = obstacles.pop(robo.target_peice_position, None)
            robo.needs_to_pick = False
            #print("Picked peice n.o {} and rotated it by {}".format(robo.target_peice_num, (robo.pickup_orientation - robo.dropping_orientation - robo.target_peice_rotation +720)%360))

        if not robo.selected_dropping_destination:
            if robo.reached:
                grid = get_grid(ARENA_SIZE)
                robo.update_position()
                dest = nearest_point(robo.get_position(), get_adjacents(grid[robo.target_peice_num], blocks))
                dest, robo.dropping_orientation = dest[:2], dest[2]
                robo.set_destination(dest)
                robo.target_peice_position = grid[robo.target_peice_num]
                robo.selected_dropping_destination = True


        #MULTIPLE ROBOTS
        ######################################
        if not robo.dropping:
            if other_robot.dropping:
                robo.blocked = True
                continue
            else:
                robo.dropping = True
                robo.blocked = False
        ######################################


        if not robo.reached_dropping_destination:
            robo.move_towards_destination()
            #print(blocks[robo.destination])
            if robo.reached:
                robo.reached_dropping_destination = True
                robo.needs_align = True
                robo.needs_to_drop = True
            if blocks[robo.destination[::-1]] < 127:
                #print("changed")
                robo.update_position()
                dest = nearest_point(robo.get_position(), get_adjacents(grid[robo.target_peice_num], blocks))
                dest, robo.dropping_orientation = dest[:2], dest[2]
                robo.set_destination(dest)
            continue
            
        if robo.needs_to_drop:
            robo.drop_block()
            placed_pecies.update({grid[robo.target_peice_num]: robo.target_peice_object})
            robo.needs_to_drop = False
            #print("Dropped peice n.o {} and at {}".format(robo.target_peice_num, robo.target_peice_position))
            robo.dropping = False
            FIVE_PLACED = True
            
        robo.needs_align = False
        robo.selected_picking_destination = False
        robo.reached_picking_destination = False
        robo.selected_dropping_destination = False
        robo.reached_dropping_destination = False

    for robo_key, robo in robots.items():
        try:
            robo.display()
            for i in robo.path:
                cv2.circle(frame, upscale((i[1], i[0])), 5, 0, thickness=-1)

            cv2.imshow(str(robo.MARKER_ID)+"-MAP", robo.blocks_map)
        except:
            pass

    for i in obstacles.keys():
        for j in get_adjacents(i, blocks):
            cv2.circle(frame, j[:2], 5, (0,255,0), thickness=-1)

    cv2.circle(frame, dest, 5, (255,255,0), thickness=-1)

    for i in grid.keys():
        cv2.circle(frame, grid[i], 5, (255, 0, 255), -1)

    '''
    #MULTIPLE ROBOTS CODE
    #########################################################################################
    for robo_key, robo in robots.items():
        posx, posy = robo.get_position(mode="head")
        cv2.circle(blocks, (posx, posy), int(ROBOT_SIZE[0]/RESIZE_SCALE), 129, thickness=-1)
        cv2.circle(frame, (posx, posy), COLLISION_DISTANCE, (0,255,0), thickness=1)
    ######################################################################################
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