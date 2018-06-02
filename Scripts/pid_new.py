import numpy as np
import cv2
import cv2.aruco as aruco
import time
#from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
import serial
from xbee import XBee

ROBOT_MARKER_ID = 11
tx=int(input("target_x="))
ty=int(input("target_y="))
point=(tx,ty)       


cap = cv2.VideoCapture(1)
       

def nothing(x):
        pass

# Instantiate an XBee device object.
def communication(inp):
        ser_port=serial.Serial('COM3',9600)
        xbee=XBee(ser_port)
        xbee.tx(dest_addr='\x00\x01',data=inp)

def get_marker_angle(point, marker_id):

   
    while(True):
        ret, frame = cap.read()

        #point = (50, 50)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters =  aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        try:
            idx = None
            for i, j in enumerate(ids):
                if j[0] == marker_id:
                    idx = i
            #print(idx)
            corner_point = corners[idx][0]
            #print(corner_point)
            #print([int(corner_point[0][0]), int(corner_point[0][1])])
            Ax = int((corner_point[3][0] + corner_point[0][0])/2)
            Ay = int((corner_point[3][1] + corner_point[0][1])/2)

            Bx = int((corner_point[2][0] + corner_point[1][0])/2)
            By = int((corner_point[2][1] + corner_point[1][1])/2)

            cv2.line(gray, point, (Ax, Ay), 255, 1)
            #cpoints = [int(corner_point[1][0]) - int(corner_point[0][0])]
            cv2.circle(gray, tuple([corner_point[0][0], 0]), 10, 255, thickness=-1)
            cv2.circle(gray, (Ax, Ay), 5, 255, thickness=-1)
            cv2.circle(gray, (Bx, By), 5, 255, thickness=-1)
            arucolength = np.sqrt((Ax-Bx)**2 + (Ay-By)**2)
            pointdist_tail = np.sqrt((Ax-point[0])**2 + (Ay-point[1])**2)
            pointdist_head = np.sqrt((Bx-point[0])**2 + (By-point[1])**2)
            m2 = (Ay-By)/(Ax-Bx)
            m1 = (Ay-point[1])/(Ax-point[0])
            theta = np.arctan((m2-m1)/(1+m1*m2))*180/np.pi
            
            if pointdist_head > np.sqrt(arucolength**2 + pointdist_tail**2):
                if theta < 0:
                    theta = 180 + theta
                elif theta >0:
                    theta = -180 + theta
                else:
                    theta = 180
        except:
                Ax,Ay,Bx,By,theta= (0,0,0,0,0)
        gray = aruco.drawDetectedMarkers(gray, corners)
        
        x,y=gray.shape
        if(point[0]>x):
                tx=x
        if(point[1]>y):
                ty=y
        cv2.imshow('image',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        break
    return Ax,Ay,Bx,By,int(theta)

class PID():

	def __init__(self):
		cv2.namedWindow('image')

		self.sample_time = 0
		self.time_now = time.time()
		self.time_previous = self.time_now

		self.set_point = (0, 0)

		self.P = 0.00
		self.I = 0.00
		self.D = 0.00
		self.error_previous = 0.0

		cv2.createTrackbar('kp','image',0,500,nothing)
		cv2.createTrackbar('ki','image',0,500,nothing)
		cv2.createTrackbar('kd','image',0,500,nothing)
		cv2.createTrackbar('sample_time','image',0,255,nothing)
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
		self.sample_time = cv2.getTrackbarPos('sample_time','image')
		self.CLOSE = cv2.getTrackbarPos(self.switch,'image')


	def get_parameters(self):
		return self.kp, self.ki, self.kd

	def compute(self):


		self.set_parameters()
		#print(self.get_parameters())

		error_now = None
		while error_now is None:
			error_now, dist = get_marker_angle(self.set_point, 11)

		de = error_now - self.error_previous

		self.time_now = time.time()
		dt = self.time_now - self.time_previous

		if dt < self.sample_time:
			return None

		self.time_previous = self.time_now

		self.P = self.kp * error_now
		self.I += self.ki * error_now * dt
		self.D = self.kd * de / dt

		output = self.P + self.I + self.D

		if output > 124:
			output = 124
			#self.I = output

		if output < -124:
			output = -124
			#self.I = output

		self.error_previous = error_now
		#print(output)

		return output, dist

pid = PID()


while(1):
               
       Ax,Ay,Bx,By,theta=get_marker_angle(point,ROBOT_MARKER_ID)
       #print(theta)
       pid.set_parameters()
       kp,ki,kd=pid.get_parameters()
       array=['<']
       array.append('T'+str(tx)+'|'+str(ty)+'|')
       array.append('P'+str(200)+'|'+str(ki)+'|'+str(kd)+'|')
       array.append('R'+str(Bx)+'|'+str(By)+'|'+str(Ax)+'|'+str(Ay)+'|'+'A'+str(theta+180)+'|')
       array.append('>')
       send_packet="".join(array)
       communication(send_packet)
       #print(send_packet)
       #array=['<']
       if pid.CLOSE != 0:
               break
        
cap.release()
cv2.destroyAllWindows()
