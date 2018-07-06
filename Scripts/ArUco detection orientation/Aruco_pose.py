# import numpy as np
import cv2
import cv2.aruco as aruco
import math


cap = cv2.VideoCapture(0)
# creating a video capture object
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    parameters = aruco.DetectorParameters_create()
    # List of Ids and the list of corners belonging to that Id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # if ArUco is detected then length of corners will be non-zero
    # if ArUco is detected then
    if len(corners) > 0:
        # from the list of corners
        for corner in corners:
            gray = cv2.circle(frame, (int(corner[0][0][0]), int(corner[0][0][1])), 2, (0, 0, 255), -1)
            gray = cv2.circle(frame, (int(corner[0][1][0]), int(corner[0][1][1])), 5, (0, 0, 255), -1)
            # center of the aruco
            cx = int(((corner[0][0][0])+(corner[0][2][0]))/2)
            cy = int(((corner[0][0][1])+(corner[0][2][1]))/2)
            # from the corner points calculate the slope
            ax = ((corner[0][1][0])-(corner[0][0][0]))
            ay = ((corner[0][1][1])-(corner[0][0][1]))
            try:
                z = ay/ax  # z is th slope
                # if slope falls in range -1 to 1 atan could not give its angle
                if abs(z) < 1:
                    z2 = math.atan(1/z)  # find the arccot
                    z1 = int(90-math.degrees(z2))  # 90 - arctan = arccot
                else:
                    z2 = math.atan(z)
                    # atan gives angle in radians, so convert it into degrees
                    z1 = int(math.degrees(z2))
                # when the angle is negative make it positive by adding 180 degrees
                if z1 < 0:
                    z1 = z1+180
                # when slope is negative angle is greater than 180
                if ay < 0:
                    z1 = z1+180
                # to get the angle from lower horizon instead of upper
                z1 = 360-z1
            except Exception as e:
                print(e)
                pass
            cv2.circle(frame, (cx, cy), 2, (0, 0, 0), -1)
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
