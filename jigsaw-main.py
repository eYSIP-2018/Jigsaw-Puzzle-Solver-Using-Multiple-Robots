import queue
import time
from itertools import combinations

import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d
import serial
from xbee import XBee

import aruco_lib
from communication import XBeeComm, Packets
from dijkstra import *
import pid_gui

from config import *


class Robot(XBeeComm, Packets, pid_gui.PID):

    def __init__(self, addr, marker_id):
        self.ADDR = addr
        self.MARKER_ID = marker_id

        self.dest_x, self.dest_y = 0, 0
        self.center_x, self.center_y, self.head_x, self.head_y, self.theta = (0,0,0,0,0)

        self.pid = pid_gui.PID('ROBOT_'+str(marker_id))

        self.packet = Packets()

        self.reached = True

    def get_position(self):
        return (self.center_x, self.center_y)

    def set_destination(self, x, y):
        self.dest_x, self.dest_y = x, y
        self.reached = False

    def get_distance(self, x, y):
        return np.sqrt((self.center_x - x)**2 + (self.center_y - y)**2)

    def send_info(self):
        global frame

        self.pid.set_parameters()

        try:
            self.center_x, self.center_y, self.head_x, self.head_y, self.theta = aruco_lib.get_marker_angle(frame, (self.dest_x, self.dest_y), self.MARKER_ID)
        except Exception as e:
            self.theta = 0
            #print(e)

        self.packet.push(('T', self.dest_x, self.dest_y))
        self.packet.push(('P', self.pid.kp, self.pid.ki, self.pid.kd))
        self.packet.push(('R', self.center_x, self.center_y, self.head_x, self.head_y))
        self.packet.push(('A', self.theta+180))

        packet_data = self.packet.createPacket()

        comm.tx(packet_data, self.ADDR)

        cv2.line(frame, (self.dest_x, self.dest_y), (self.center_x, self.center_y), (255, 255, 255), 1)
        cv2.circle(frame, (self.dest_x, self.dest_y), 5, (0,0,255), thickness=-1)
        cv2.circle(frame, (self.center_x, self.center_y), 5, (0,255,0), thickness=-1)
        cv2.circle(frame, (self.head_x, self.head_y), 5, (0,255,0), thickness=-1)

        self.packet.resetData()

        head_dist = self.get_distance(self.dest_x, self.dest_y)

        if head_dist <= 20:
            self.reached = True


def mouse_events(event, x, y, flags, param):
    global dest
    global voronoi_vertices
    global MODE
    global src

    if event == cv2.EVENT_LBUTTONDBLCLK and MODE == 0:
        prev_dist = 9999
        for i in voronoi_vertices:
            dist = distance(i, (x,y))
            if dist < prev_dist:
                prev_dist = dist
                src = i

    if event == cv2.EVENT_LBUTTONDBLCLK and MODE == 1:
        prev_dist = 9999
        for i in voronoi_vertices:
            dist = distance(i, (x,y))
            if dist < prev_dist:
                prev_dist = dist
                dest = i


global frame
global src
global dest
global MODE
global voronoi_vertices

MODE = 0
dest = (0, 0)
src = (0,0)

comm = XBeeComm()

robots = [
    Robot(ROBOT_1_XBee_ADDR, ROBOT_1_MARKER_ID),
    Robot(ROBOT_2_XBee_ADDR, ROBOT_2_MARKER_ID),
    ]

corners = []
edges = []
unwanted_vertices = []
obstacle_region = {}

filename = 'blocks1.png'
img = cv2.imread(filename)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

_, contours, hierarchy = cv2.findContours(gray, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

(max_y, max_x, _) = img.shape

sep_x = int(max_x/2)
sep_y = int(max_y/2)

for i in range(0, max_x + sep_x, sep_x):
    for j in range(0, max_y + sep_y, sep_y):
        corners.append((i,j))

for i, c in enumerate(contours[1:]):
    
    rect  = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    box = [tuple(i) for i in box]

    _ = [corners.append(i) for i in box]

    cx = 0; cy = 0

    for i in box:
        cx += i[0]
        cy += i[1]

    cx = int(cx/4)
    cy = int(cy/4)

    max_dist = 0
    for k in box:
        dist = abs(distance(k,(cx, cy)))
        if max_dist < dist:
            max_dist = dist

    obstacle_region[(cx,cy)] = max_dist + OBSTACLE_SAFETY_BOUNDARY
    max_dist = 0

    #corners.append((cx,cy))

vor = Voronoi(corners)

voronoi_vertices = [(int(i[0]), int(i[1])) for i in vor.vertices]

(max_y, max_x, _) = img.shape

for i in voronoi_vertices:
    if i[0] > max_x or i[1] > max_y or 0 > i[0] or 0 > i[1]:
        unwanted_vertices.append(i)

for (i,j) in vor.ridge_vertices:
    if i < 0 and j < 0:
        continue

    inside = False

    for k in obstacle_region.keys():
        if distance(voronoi_vertices[i], k) <= obstacle_region[k]:
            inside = True
        if distance(voronoi_vertices[j], k) <= obstacle_region[k]:
            inside = True
        if inside == True:
            break

    if not inside:
        if (voronoi_vertices[i] in unwanted_vertices) or (voronoi_vertices[j] in unwanted_vertices):
            continue
        v1 = voronoi_vertices[i]
        v2 = voronoi_vertices[j]
        edges.append((v1,v2))

cap = cv2.VideoCapture('/dev/video0')

cv2.namedWindow('image')
cv2.setMouseCallback('image', mouse_events)

while True:
    ret, frame = cap.read()

    try:
        frame = aruco_lib.get_arena(frame)
    except:
        pass

    img = cv2.imread(filename)

    for i in obstacle_region.keys():
        cv2.circle(img, i, int(obstacle_region[i]), (0,255,0), thickness=1)

    for i in voronoi_vertices[:]:
        cv2.circle(img, i, 5, (127,127,127), thickness=-1)
    
    for i in edges:
        cv2.line(img, i[0], i[1], (127,127,127), 3)

    if MODE >1:
        graph = create_graph(edges)
        path = get_path(graph, src, dest)

        if MODE == 2:
            robot_path = path[:]

        curr_node = path[0]
        path = path[1:]

        for i in path:
            cv2.line(img, i, curr_node, (255,0,0), 3)
            curr_node = i
        
        for i in path:
            cv2.circle(img, i, 5, (0,255,255), thickness=-1)
            curr_node = i

    try:
        cv2.circle(img, src, 5, (0,255,0), thickness=-1)
        cv2.circle(img, dest, 5, (0,0,255), thickness=-1)
    except:
        pass

    if MODE > 2 and len(robot_path) != 0:

        if robots[0].reached:
            curr_checkpoint = robot_path[0]

            try:
                robot_path = robot_path[1:]
            except:
                robot_path = []
            
            robots[0].set_destination(*curr_checkpoint)

    if MODE > 3 or MODE < 0:
        MODE = 0
    
    if not robots[0].reached:
        robots[0].send_info()
        cv2.circle(img, curr_checkpoint, 5, (0,0,255), thickness=-1)

    cv2.circle(img, robots[0].get_position(), 5, (0,255,0), thickness=-1)
    
    cv2.imshow('image',img)
    cv2.imshow('frame',frame)

    opt = cv2.waitKey(1) & 0xFF

    if opt == ord('q'):
        break
    elif opt == ord('d'):
        MODE += 1

cap.release()
cv2.destroyAllWindows()
