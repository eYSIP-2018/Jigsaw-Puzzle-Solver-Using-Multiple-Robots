import cv2
import cv2.aruco as aruco
import numpy as np

from config import *

def get_aruco_markers(img):
    """Finds all the aruco markers in the given image
    Arguments:
        img (numpy.array): image in which the arucos are to be searched
    Returns:
        corners, ids (list, list): a list of list of corner points for each aruco, list consisting of ids of each aruco
    Raises:
        "no arucos found" exception when no arucos are found in the image 
    """    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
    parameters =  aruco.DetectorParameters_create()

    try:
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
        return corners, ids
    except:
        raise Exception("no arucos found")

def get_marker_by_id(corners, ids, aruco_id):
    """Finds the corners of the given aruco id
    Arguments:
        corners (list): a list of list of corner points for each aruco
        ids (list): list consisting of ids of each aruco
        aruco_id: query aruco id
    Returns:
        corners (list): corners of the queried aruco id
    Raises:
        rasies exception if the queried aruco id is not in the ids list
    """
    try:
        idx = None
        for i, j in enumerate(ids):
            if j[0] == aruco_id:
                idx = i
        return corners[idx][0]
    except:
        raise Exception("get_marker_by_id: aruco with id " + str(aruco_id) + " not found")

def get_marker_angle(img, point, marker_id):
    """Finds the pose and orientation of the marker
    Arguments:
        img (numpy.array): image
        point ((int, int)): cordinates of the given point
        marker_id (int): id of the queried aruco marker
    Returns:
        (Cx,Cy),(Hx,Hy),theta ((int, int), (int, int), int): center of the aruco, head of the aruco, the angle between aruco and line joining the point and aruco center
    """
    try:
        #convert image to gray scale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #find all arucos in the image
        corners, ids = get_aruco_markers(gray)

        #find aruco with given maker id
        corner_point = get_marker_by_id(corners, ids, marker_id)

        #tail cordinates of the aruco
        Tx = int((corner_point[3][0] + corner_point[0][0])/2)
        Ty = int((corner_point[3][1] + corner_point[0][1])/2)

        #head cordinates of the aruco
        Hx = int((corner_point[2][0] + corner_point[1][0])/2)
        Hy = int((corner_point[2][1] + corner_point[1][1])/2)

        #center cordinates of the aruco
        Cx = int((Tx+Hx)/2)
        Cy = int((Ty+Hy)/2)

        #length of the aruco
        arucolength = np.sqrt((Cx-Hx)**2 + (Cy-Hy)**2)

        #distance of point from aruco center
        center_dist = np.sqrt((Cx-point[0])**2 + (Cy-point[1])**2)

        #distance of point from head of aruco
        head_dist = np.sqrt((Hx-point[0])**2 + (Hy-point[1])**2)

        #find the slope of aruco
        m2 = (Cy-Hy)/(Cx-Hx + 0.001)

        #find the slope of line connecting the aruco center and the point
        m1 = (Cy-point[1])/(Cx-point[0] + 0.001)

        #find the angle between aruco and line joining the point and aruco center
        theta = np.arctan((m2-m1)/(1+m1*m2+0.001))*180/np.pi

        #find if the angle is obtuse
        if head_dist > np.sqrt(arucolength**2 + center_dist**2):
            #add 180 if the angle negative
            if theta < 0:
                theta = 180 + theta
            #substract 180 if the angle is positive
            elif theta > 0:
                theta = -180 + theta
            else:
                theta = 180
    except Exception as e:
        raise Exception(e)

    return (Cx,Cy),(Hx,Hy),int(theta)
