import numpy as np
import cv2
import cv2.aruco as aruco
import math
from slope_orientation import slope
cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    parameters =  aruco.DetectorParameters_create()
    x,y,_=frame.shape
    
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #print ((ids))
    if len(corners)>0:
        for corner in corners:
            
            
            axl =int(((corner[0][3][0])+(corner[0][0][0]))/2) #x-mid point of left side of aruco 
            axr =int(((corner[0][1][0])+(corner[0][2][0]))/2) #x-mid point of right side of aruco
            ayl =int(((corner[0][3][1])+(corner[0][0][1]))/2) #y-mid point of right side of aruco
            ayr =int(((corner[0][1][1])+(corner[0][2][1]))/2) #y-mid point of right side of aruco
            gray=cv2.circle(gray,(axl,ayl),5,(255,255,255),-1)
           
            gray=cv2.circle(gray,(axr,ayr),5,(255,255,255),-1)
            
            cx  =int(((corner[0][0][0])+(corner[0][2][0]))/2) #x-mid point of diagonal
            cy  =int(((corner[0][0][1])+(corner[0][2][1]))/2) #y-mid point of diagonal
            z1=slope(axl,ayl,axr,ayr)      
            cv2.circle(gray,(cx,cy),2,(0,0,0),-1)
            gray=cv2.putText(gray,str(cx)+","+str(cy)+","+str(z1)+"deg",(cx,cy), 1, 2,(255,255,255),2,cv2.LINE_AA)
        for i in range(len(ids)):
            gray=cv2.putText(gray,str(ids[i-1]),(int(corners[i-1][0][0][0]),int(corners[i-1][0][0][1])), 1, 2,(0,0,0),2,cv2.LINE_AA)
            


    gray = aruco.drawDetectedMarkers(gray, corners)
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
