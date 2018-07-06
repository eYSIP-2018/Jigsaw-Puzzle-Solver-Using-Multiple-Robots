import numpy as np
import cv2
import cv2.aruco as aruco
import math
import time 
 
cap = cv2.VideoCapture(1)
 
while(True):
    ret, frame = cap.read()
    print ret
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters =  aruco.DetectorParameters_create()
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    
    try:
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        print ids
        for i, j in enumerate(ids):
            if j[0] == 8:
                print corners[i]
                for k in corners[i][0]:
	                x1 += int(k[0])
	                y1 += int(k[1])
            if j[0] == 11:
                print corners[i]
                for k in corners[i][0]:
	                x2 += int(k[0])
	                y2 += int(k[1])
        x1 = x1/4
        x2 = x2/4
        y1 = y1/4
        y2 = y2/4
        cv2.imshow('frame',gray[y1:y2,x1:x2])
    except:
        pass
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
