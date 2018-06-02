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


class XBeeComm:

    def __init__(self, com='COM3', baud=9600):
        self.xbee = XBee(serial.Serial(com,baud))
        
    def tx(self, data, dest='\x00\x01'):
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
        packet = "<" + "".join(self.data) + ">"
        return packet


class PID:

    def __init__(self):
        cv2.namedWindow('image')

        cv2.createTrackbar('kp','image',0,500,nothing)
        cv2.createTrackbar('ki','image',0,500,nothing)
        cv2.createTrackbar('kd','image',0,500,nothing)

        self.switch = '0 : ON \n1 : OFF'
        cv2.createTrackbar(self.switch, 'image',0,1,nothing)

        self.kp = cv2.getTrackbarPos('kp','image')
        self.ki = cv2.getTrackbarPos('ki','image')
        self.kd = cv2.getTrackbarPos('kd','image')
        self.CLOSE = cv2.getTrackbarPos(self.switch,'image')

    def setpoint(self, set_point):
        self.set_point = set_point

    def set_parameters(self):
        self.kp = cv2.getTrackbarPos('kp','image')
        self.ki = cv2.getTrackbarPos('ki','image')
        self.kd = cv2.getTrackbarPos('kd','image')
        self.CLOSE = cv2.getTrackbarPos(self.switch,'image')

    def get_parameters(self):
        return self.kp, self.ki, self.kd

def get_aruco_markers(img):
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

def get_marker_angle(frame, point, marker_id):
    while(True):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            corners, ids = get_aruco_markers(gray)
            corner_point = get_marker_by_id(corners, ids, marker_id)

            Tx = int((corner_point[3][0] + corner_point[0][0])/2)
            Ty = int((corner_point[3][1] + corner_point[0][1])/2)

            Hx = int((corner_point[2][0] + corner_point[1][0])/2)
            Hy = int((corner_point[2][1] + corner_point[1][1])/2)

            cv2.line(gray, point, (Tx, Ty), 255, 1)
            cv2.circle(gray, (Tx, Ty), 5, 255, thickness=-1)
            cv2.circle(gray, (Hx, Hy), 5, 255, thickness=-1)

            arucolength = np.sqrt((Tx-Hx)**2 + (Ty-Hy)**2)
            tail_dist = np.sqrt((Tx-point[0])**2 + (Ty-point[1])**2)
            head_dist = np.sqrt((Hx-point[0])**2 + (Hy-point[1])**2)

            m2 = (Ty-Hy)/(Tx-Hx)
            m1 = (Ty-point[1])/(Tx-point[0])

            theta = np.arctan((m2-m1)/(1+m1*m2))*180/np.pi

            if head_dist > np.sqrt(arucolength**2 + tail_dist**2):
                if theta < 0:
                    theta = 180 + theta
                elif theta >0:
                    theta = -180 + theta
                else:
                    theta = 180

            gray = aruco.drawDetectedMarkers(gray, corners)
        except:
            Tx,Ty,Hx,Hy,theta= (0,0,0,0,0)

        cv2.imshow('image',gray)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        break

    return Tx,Ty,Hx,Hy,int(theta)

def nothing(x):
    pass


cap = cv2.VideoCapture('/dev/video1')

dest_x = int(input("target_x="))
dest_y = int(input("target_y="))
point=(dest_x, dest_y)

pid = PID()
comm = XBeeComm()
packet = Packets()

while(1):
    ret, frame = cap.read()
    frame = get_arena(frame)

    Tx,Ty,Hx,Hy,theta = get_marker_angle(frame, point,ROBOT_1_MARKER_ID)

    pid.set_parameters()

    packet.push(('T', dest_x, dest_y))
    packet.push(('P', pid.kp, pid.ki, pid.kd))
    packet.push(('R', Hx, Hy, Tx, Ty))
    packet.push(('A', theta+180))

    packet_data = packet.createPacket()

    comm.tx(packet_data)

    packet.resetData()

    if pid.CLOSE != 0:
            break
        
cap.release()
cv2.destroyAllWindows()
