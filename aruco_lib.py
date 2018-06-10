import cv2
import cv2.aruco as aruco
import numpy as np

from config import *

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

def get_marker_angle(img, point, marker_id):
    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids = get_aruco_markers(gray)
        corner_point = get_marker_by_id(corners, ids, marker_id)

        Tx = int((corner_point[3][0] + corner_point[0][0])/2)
        Ty = int((corner_point[3][1] + corner_point[0][1])/2)

        Hx = int((corner_point[2][0] + corner_point[1][0])/2)
        Hy = int((corner_point[2][1] + corner_point[1][1])/2)

        Cx = int((Tx+Hx)/2)
        Cy = int((Ty+Hy)/2)

        arucolength = np.sqrt((Cx-Hx)**2 + (Cy-Hy)**2)
        center_dist = np.sqrt((Cx-point[0])**2 + (Cy-point[1])**2)
        head_dist = np.sqrt((Hx-point[0])**2 + (Hy-point[1])**2)

        m2 = (Cy-Hy)/(Cx-Hx)
        m1 = (Cy-point[1])/(Cx-point[0])

        theta = np.arctan((m2-m1)/(1+m1*m2))*180/np.pi

        if head_dist > np.sqrt(arucolength**2 + center_dist**2):
            if theta < 0:
                theta = 180 + theta
            elif theta > 0:
                theta = -180 + theta
            else:
                theta = 180
    except Exception as e:
        raise Exception(e)

    return Cx,Cy,Hx,Hy,int(theta)