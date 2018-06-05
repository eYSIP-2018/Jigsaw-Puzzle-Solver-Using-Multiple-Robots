import numpy as np
import cv2
import cv2.aruco as aruco
import time
import serial
from xbee import XBee

UPPER_BOUND_MARKER_ID = 8
LOWER_BOUND_MARKER_ID = 9

ROBOT_1_MARKER_ID = 11
ROBOT_1_XBee_ADDR = '\x00\x01'

ROBOT_2_MARKER_ID = 7
ROBOT_2_XBee_ADDR = '\x00\x03'


class XBeeComm:

    def __init__(self, com='COM3', baud=9600):
        #self.xbee = XBee(serial.Serial(com,baud))
        pass
        
    def tx(self, data, dest='\x00\x01'):
        #print(data); return
        self.xbee.tx(dest_addr=dest,data=data)


class Packets:

    def __init__(self):
        self.data = []

    def push(self, data):

        if type(data) is tuple:
            for i in data:
                self.push(i)
            return

        data = str(data)
        if data.isdigit():
            data = data + '|'

        self.data.append(data)

    def resetData(self):
        self.data = []

    def createPacket(self):
        packet = '<' + ''.join(self.data) + '>'
        return packet


class PID:

    def __init__(self, robot_id):

        self.ROBOT_ID = robot_id

        cv2.namedWindow(self.ROBOT_ID)

        cv2.createTrackbar('kp',self.ROBOT_ID,0,500,nothing)
        cv2.createTrackbar('ki',self.ROBOT_ID,0,500,nothing)
        cv2.createTrackbar('kd',self.ROBOT_ID,0,500,nothing)

        self.kp = cv2.getTrackbarPos('kp',self.ROBOT_ID)
        self.ki = cv2.getTrackbarPos('ki',self.ROBOT_ID)
        self.kd = cv2.getTrackbarPos('kd',self.ROBOT_ID)

    def setpoint(self, set_point):
        self.set_point = set_point

    def set_parameters(self):
        self.kp = cv2.getTrackbarPos('kp',self.ROBOT_ID)
        self.ki = cv2.getTrackbarPos('ki',self.ROBOT_ID)
        self.kd = cv2.getTrackbarPos('kd',self.ROBOT_ID)

    def get_parameters(self):
        return self.kp, self.ki, self.kd


class Robot(XBeeComm, Packets, PID):

    def __init__(self, addr, marker_id):
        self.ADDR = addr
        self.MARKER_ID = marker_id

        self.dest_x, self.dest_y = 0, 0
        self.tail_x, self.tail_y, self.head_x, self.head_y, self.theta = (0,0,0,0,0)

        self.pid = PID('ROBOT_'+str(marker_id))

        self.packet = Packets()

    def set_destination(self, x, y):
        self.dest_x, self.dest_y = x, y

    def get_distance(self, x, y):
        return np.sqrt((self.head_x - x)**2 + (self.head_y - y)**2)

    def send_info(self):
        global frame

        self.pid.set_parameters()

        try:
            self.tail_x, self.tail_y, self.head_x, self.head_y, self.theta = get_marker_angle(frame, (self.dest_x, self.dest_y), self.MARKER_ID)
        except:
            self.theta = 0

        self.packet.push(('T', self.dest_x, self.dest_y))
        self.packet.push(('P', self.pid.kp, self.pid.ki, self.pid.kd))
        self.packet.push(('R', self.tail_x, self.tail_y, self.head_x, self.head_y))
        self.packet.push(('A', self.theta+180))

        packet_data = self.packet.createPacket()

        comm.tx(packet_data, self.ADDR)

        cv2.line(frame, (self.dest_x, self.dest_y), (self.tail_x, self.tail_y), (255, 255, 255), 1)
        cv2.circle(frame, (self.dest_x, self.dest_y), 5, (0,0,255), thickness=-1)
        cv2.circle(frame, (self.tail_x, self.tail_y), 5, (0,255,0), thickness=-1)
        cv2.circle(frame, (self.head_x, self.head_y), 5, (0,255,0), thickness=-1)

        self.packet.resetData()


def get_aruco_markers(img):
    global frame    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters =  aruco.DetectorParameters_create()

    try:
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
        return corners, ids
    except:
        raise Exception("no arucos found")

def get_arena(img):
    corners, ids= get_aruco_markers(img)
    ((x1,y1), (x2,y2)) = ((0,0), (0,0))
    try:
        upper_corner_pts = get_marker_by_id(corners, ids, UPPER_BOUND_MARKER_ID)
        lower_corner_pts = get_marker_by_id(corners, ids, LOWER_BOUND_MARKER_ID)

        for i in upper_corner_pts:
            x1 += int(i[0])
            y1 += int(i[1])

        for i in lower_corner_pts:
            x2 += int(i[0])
            y2 += int(i[1])

        x1 = int(x1/4)
        x2 = int(x2/4)
        y1 = int(y1/4)
        y2 = int(y2/4)

        return img[y1:y2,x1:x2]
    except:
        return img

def get_marker_by_id(corners, ids, aruco_id):
    try:
        idx = None
        for i, j in enumerate(ids):
            if j[0] == aruco_id:
                idx = i
        return corners[idx][0]
    except:
        raise Exception("aruco with id " + str(aruco_id) + " not found")

def get_marker_angle(img, point, marker_id):
    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids = get_aruco_markers(gray)
        corner_point = get_marker_by_id(corners, ids, marker_id)

        Tx = int((corner_point[3][0] + corner_point[0][0])/2)
        Ty = int((corner_point[3][1] + corner_point[0][1])/2)

        Hx = int((corner_point[2][0] + corner_point[1][0])/2)
        Hy = int((corner_point[2][1] + corner_point[1][1])/2)

        arucolength = np.sqrt((Tx-Hx)**2 + (Ty-Hy)**2)
        tail_dist = np.sqrt((Tx-point[0])**2 + (Ty-point[1])**2)
        head_dist = np.sqrt((Hx-point[0])**2 + (Hy-point[1])**2)

        m2 = (Ty-Hy)/(Tx-Hx)
        m1 = (Ty-point[1])/(Tx-point[0])

        theta = np.arctan((m2-m1)/(1+m1*m2))*180/np.pi

        if head_dist > np.sqrt(arucolength**2 + tail_dist**2):
            if theta < 0:
                theta = 180 + theta
            elif theta > 0:
                theta = -180 + theta
            else:
                theta = 180
    except Exception as e:
        raise Exception(e)

    return Tx,Ty,Hx,Hy,int(theta)

def mouse_events(event, x, y, flags, param):
    global dest_x
    global dest_y
    
    if event == cv2.EVENT_LBUTTONDBLCLK:
        prev_dist = 99999
        robo = None
        for i in robots:
            dist = i.get_distance(x,y)
            if dist < prev_dist:
                prev_dist = dist
                robo = i
        
        robo.set_destination(x,y)

def nothing(x):
    pass


global frame

cap = cv2.VideoCapture('/dev/video1')

cv2.namedWindow('image')
cv2.setMouseCallback('image', mouse_events)

comm = XBeeComm()

robots = [
    Robot(ROBOT_1_XBee_ADDR, ROBOT_1_MARKER_ID),
    Robot(ROBOT_2_XBee_ADDR, ROBOT_2_MARKER_ID),
    ]

while(1):
    ret, frame = cap.read()
    try:
        frame = get_arena(frame)
    except:
        pass

    for i in robots:
        i.send_info()

    cv2.imshow('image',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
