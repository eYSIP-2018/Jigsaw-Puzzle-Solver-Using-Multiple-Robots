import numpy as np
import cv2
import cv2.aruco as aruco
import math
from slope_orientation import slope

def angle_required(tx,ty,frame,corners,ids):
    if len(corners)>0:
            for corner in corners:
                
                gray=cv2.circle(frame,(int(corner[0][0][0]),int(corner[0][0][1])),2,(0,0,255),-1)
               
                gray=cv2.circle(frame,(int(corner[0][1][0]),int(corner[0][1][1])),5,(0,0,255),-1)

                
                #######################################################################################

                
                cx =int(((corner[0][0][0])+(corner[0][2][0]))/2)
                cy =int(((corner[0][0][1])+(corner[0][2][1]))/2)


                
                ########################################################################################
                axl =int(((corner[0][3][0])+(corner[0][0][0]))/2) #x-mid point of left side of aruco 
                axr =int(((corner[0][1][0])+(corner[0][2][0]))/2) #x-mid point of right side of aruco
                ayl =int(((corner[0][3][1])+(corner[0][0][1]))/2) #y-mid point of right side of aruco
                ayr =int(((corner[0][1][1])+(corner[0][2][1]))/2) #y-mid point of right side of aruco
                #########################################################################################

                
                gray=cv2.circle(gray,(axl,ayl),5,(255,255,255),-1)
               
                gray=cv2.circle(gray,(axr,ayr),5,(255,255,255),-1)


                
                ###############################################################################################
                #z1=slope(int(corner[0][0][0]),int(corner[0][0][1]),int(corner[0][1][0]),int(corner[0][1][1]))
                
                z1=slope(axl,ayl,axr,ayr) 
                angle_to_move=slope(cx,cy,tx,ty)
                
                
                #############################################################################################
                cv2.circle(frame,(cx,cy),2,(0,0,0),-1)
                cv2.line(frame,(cx,cy),(tx,ty),(255,0,0),3)
                cv2.putText(frame,str(angle_to_move)+"deg",(tx,ty),1,2,(255,255,0),2,cv2.LINE_AA)
                frame=cv2.putText(frame,str(cx)+","+str(cy)+","+str(z1)+"deg",(cx,cy), 1, 2,(0,255,0),2,cv2.LINE_AA)
            for i in range(len(ids)):
                frame=cv2.putText(frame,str(ids[i-1][0]),(int(corners[i-1][0][0][0]),int(corners[i-1][0][0][1])), 1, 2,(255,0,0),2,cv2.LINE_AA)
            angle=angle_to_move-z1
    else:
        angle= 0

    return angle
