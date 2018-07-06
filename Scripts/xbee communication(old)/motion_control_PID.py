import cv2
import cv2.aruco as aruco
import numpy as np
import serial
def nothing(x):
    pass

from Angle import angle_required
from xbeeremotedevice import communication
last_angle=0
initial_speed=125
tx=int(input("target_x="))
ty=int(input("target_y="))


cv2.namedWindow('track')
cv2.createTrackbar('kp','track',0,100,nothing)
cv2.createTrackbar('kd','track',0,100,nothing)
cv2.createTrackbar('ki','track',0,100,nothing)
cap = cv2.VideoCapture(1)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    angle=angle_required(tx,ty,frame,corners,ids)
    if angle==0:
        last_angle=0
    #print(angle)
   
    #cv2.createTrackbar('speed','track',0,255,nothing)
    
    kp=cv2.getTrackbarPos('kp','track')
    kd=cv2.getTrackbarPos('kd','track')
    ki=cv2.getTrackbarPos('ki','track')
    #initial_speed=cv2.getTrackbarPos('speed','track')
    p=0.1*kp
    d=0.01*kd
    i=0.0001*ki
    new_speed=int( angle*p+d*(angle-last_angle))
    if new_speed>initial_speed:
        new_speed=initial_speed
    if new_speed<-initial_speed:
        new_speed=-initial_speed

    
    communication(chr(new_speed+125))
    lastangle=angle
    frame = aruco.drawDetectedMarkers(frame, corners)
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


