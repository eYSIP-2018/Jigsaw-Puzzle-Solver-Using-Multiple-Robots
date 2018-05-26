import numpy as np
import cv2
import cv2.aruco as aruco
import math

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
            
            gray=cv2.circle(gray,(int(corner[0][0][0]),int(corner[0][0][1])),2,(0,0,255),-1)
           
            gray=cv2.circle(gray,(int(corner[0][1][0]),int(corner[0][1][1])),5,(0,0,255),-1)
            axl =int(((corner[0][3][0])+(corner[0][0][0]))/2)
            axr =int(((corner[0][1][0])+(corner[0][2][0]))/2)
            ayl =int(((corner[0][3][1])+(corner[0][0][1]))/2)
            ayr =int(((corner[0][2][0])+(corner[0][0][0]))/2)
            ax  =axr-axl
            ay  =ayr-ayl
            cx  =int(((corner[0][0][0])+(corner[0][2][0]))/2)
            cy  =int(((corner[0][0][1])+(corner[0][2][1]))/2)
            z   =ay/ax
            
            if abs(z)<1:
                z2=math.atan(1/z)
                z1=int(90-math.degrees(z2))
            else:
                z2=math.atan(z)
                z1=int(math.degrees(z2))
            if z1<0:
                z1=z1+180
            if ay<0:
                z1=z1+180
                    
            cv2.circle(gray,(cx,cy),2,(0,0,0),-1)
            gray=cv2.putText(gray,str(cx)+" "+str(cy)+" "+str(z1),(cx,cy), 1, 2,(0,0,0),2,cv2.LINE_AA)
        for i in range(len(ids)):
            gray=cv2.putText(gray,str(ids[i-1]),(int(corners[i-1][0][0][0]),int(corners[i-1][0][0][1])), 1, 2,(0,0,0),2,cv2.LINE_AA)
            


    gray = aruco.drawDetectedMarkers(gray, corners)
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
