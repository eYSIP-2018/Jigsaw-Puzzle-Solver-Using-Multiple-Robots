# import numpy as np
import cv2
import cv2.aruco as aruco
# import math
from slope_orientation import slope


# give the target point co-ordinate
tx = int(input("target_x="))
ty = int(input("target_y="))

cap = cv2.VideoCapture(0)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # when ArUco is detected
    if len(corners) > 0:
        for corner in corners:
            gray = cv2.circle(frame, (int(corner[0][0][0]), int(corner[0][0][1])), 2, (0, 0, 255), -1)
            gray = cv2.circle(frame, (int(corner[0][1][0]), int(corner[0][1][1])), 5, (0, 0, 255), -1)
            # center of aruco
            cx = int(((corner[0][0][0])+(corner[0][2][0]))/2)
            cy = int(((corner[0][0][1])+(corner[0][2][1]))/2)
            # orientation of robot
            z1 = slope(int(corner[0][0][0]), int(corner[0][0][1]), int(corner[0][1][0]), int(corner[0][1][1]))
            angle_to_move = slope(cx, cy, tx, ty)
            # print(angle_to_move)
            cv2.circle(frame, (cx, cy), 2, (0, 0, 0), -1)
            cv2.line(frame, (cx, cy), (tx, ty), (255, 0, 0), 3)
            cv2.putText(frame, str(angle_to_move)+"deg", (tx, ty), 1, 2, (255, 255, 0), 2, cv2.LINE_AA)
            frame = cv2.putText(frame, str(cx)+","+str(cy)+","+str(z1)+"deg", (cx, cy), 1, 2, (0, 255, 0), 2, cv2.LINE_AA)
        for i in range(len(ids)):
            frame = cv2.putText(frame, str(ids[i-1][0]), (int(corners[i-1][0][0][0]), int(corners[i-1][0][0][1])), 1, 2, (255, 0, 0), 2, cv2.LINE_AA)
    frame = aruco.drawDetectedMarkers(frame, corners)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
