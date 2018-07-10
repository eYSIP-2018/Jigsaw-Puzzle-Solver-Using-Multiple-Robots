import math

import cv2
import numpy as np
import imutils

from config import *
from misc import *

def get_frame(cap):
    """Returns a uindistorted frame removing fish eye effect
    Returns:
        dst (numpy.array): undistorted image of the arena
    """

    #camera matrix for camera calibration
    mtx = np.array(np.mat("588.4525598886621, 0, 301.8008794717551; 0, 588.9763096391521, 242.617026416902; 0, 0, 1"))

    #distrotion coefficients for camera calibration
    dist = np.array(np.mat("-0.4351555722591889, 0.2082765081608728, -0.006072767012672472, 0.008139871640987759, 0"))

    #get image frame from the camera
    ret, frame = cap.read()

    return frame

    h,  w = frame.shape[:2]

    #get the new optimal camera matrix and the roi which can be used to crop the result
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h))

    #get the undistroted image
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    x,y,w,h = roi

    #get the cropped image
    dst = dst[y:y+h, x:x+w]
    h, w = dst.shape[:2]

    #furthur crop the image to reduce the size of arena
    dst = dst[int(h/7):int(h*6/7), int(w/7):int(w*6/7)]

    #resize the arena to ARENA_SIZE
    dst = cv2.resize(dst, ARENA_SIZE, interpolation= cv2.INTER_CUBIC)

    return dst

def get_obstacles(image):
    """Returns a dictionary containing the position of the obstacles as keys and width, height of obstacles as values
    and a dictionary containg the roi of puzzle peices their positions as keys
    Arguments:
        image (numpy.array): digital image of the arena configuration
    Returns:
        obstales, blocks_roi (dict, dict): a dictionary containing the position of the obstacles as keys and (width, height) of obstacles as values, 
            a dictionary containg the roi of puzzle peices their positions as keys
    """

    ih, iw = image.shape[:2]
    image_copy = image.copy()

    #resize the image to the size of arena
    image = cv2.resize(image, ARENA_SIZE, interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    #replace all black pixels to white pixels
    gray[np.where(gray == 0)]= 255

    #get the thresholded binary image
    ret,threshold = cv2.threshold(gray,200,255,cv2.THRESH_BINARY_INV)

    #find all the countours in the binary image
    _, contours, heiarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    cont = []

    #create a mask to draw contours on
    blocks = mask = np.zeros(threshold.shape[:2], np.uint8)

    #create a dictionary to hold image roi of all puzzle peices
    blocks_roi = {}

    #iterate through all contours
    for i, c in enumerate(contours[1:]):

        #find the minimum area fitting rectangle of the contour
        rect  = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        #create the copy of the mask
        mask_copy = mask.copy()

        #draw the rectangle on the mask
        cv2.drawContours(mask_copy, [box], -1, (255,255,255), 3)

        #floodfill the rectangle
        cv2.floodFill(mask_copy, None, (0,0), 255)
        mask_inv = cv2.bitwise_not(mask_copy)
        blocks = cv2.add(blocks, mask_inv)

    _, contours, heiarchy = cv2.findContours(blocks, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    obstacles = {}

    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        obstacles.update({(int(x+w/2), int(y+h/2)): BLOCK_SIZE})
        #obstacles.update({(int(x+w/2), int(y+h/2)): (w, h)}) # for unknown block sizes
        bottom_r = remap((x+w, y+h), ARENA_SIZE, (iw,ih))
        top_l = remap((x, y), ARENA_SIZE, (iw,ih))
        blocks_roi.update({(int(x+w/2), int(y+h/2)): image_copy[top_l[1]:bottom_r[1], top_l[0]:bottom_r[0]]})

    return obstacles, blocks_roi

def get_obstacles_map(obstacles, placed_pecies):
    """Creates a obstacle map (image with pixel values less than 127 at places where there are obstacles)
    Arguments:
        obstacles (dict): a dictionary containing the position of the obstacles as keys and (width, height) of obstacles as values
    Returns:
        blocks (numpy.array): an image containing the obstacles as black pixels with safety area gray pixels
    """
    
    #create a mask image to draw the obstacles on
    blocks = np.zeros(ARENA_SIZE[::-1], np.uint8)

    #get the grid points where the robot needs to placed
    grid = get_grid(ARENA_SIZE)

    #draw the obstacles and their safety region on the map
    for i in obstacles.keys():
        cv2.circle(blocks, i, int(CIRCULAR_SAFETY_FACTOR*BLOCK_SIZE[0]), 129, -1)
        cv2.rectangle(blocks, (i[0]-int(obstacles[i][0]/4), i[1]-int(obstacles[i][1]/4)), (i[0]+int(obstacles[i][0]/4), i[1]+int(obstacles[i][1]/4)), 255, -1)

    #draw the obstacles and their safety region on the map
    for i in placed_pecies.keys():
        try:
            if not i == grid[5]:
                cv2.circle(blocks, i, int(CIRCULAR_SAFETY_FACTOR*BLOCK_SIZE[0]), 129, -1)
            else:
                cv2.rectangle(blocks, (int(i[0]-7.4*placed_pecies[i][0]/4), int(i[1]-7.4*placed_pecies[i][1]/4)),
                    (int(i[0]+7.4*placed_pecies[i][0]/4), int(i[1]+7.4*placed_pecies[i][1]/4)), 129, -1)
            cv2.rectangle(blocks, (i[0]-int(placed_pecies[i][0]/4), i[1]-int(placed_pecies[i][1]/4)), (i[0]+int(placed_pecies[i][0]/4), i[1]+int(placed_pecies[i][1]/4)), 255, -1)
        except Exception as e:
            print(e)

    return cv2.bitwise_not(blocks)

def get_matches(jig_sol, blocks_roi):
    """Returns dictionary containing peice position as keys and (peice number, rotation of block) as values
    Arguments:
        jig_sol (numpy.array): image of jigsaw solution
        blocks_roi (dict): dictionary containing position of blocks as keys and roi of the block as values
    Returns:
        match_data (dict): dictionary containing peice position as keys and (peice number, rotation of block) as values
    """

    match_data = {}
    height, width,= jig_sol.shape
    font = cv2.FONT_HERSHEY_SIMPLEX

    #identify the puzzle peice number based on the peice block position in solution image
    identity = {
        (1, 1): 1,
        (2, 1): 2,
        (3, 1): 3,
        (1, 2): 4,
        (2, 2): 5,
        (3, 2): 6,
        (1, 3): 7,
        (2, 3): 8,
        (3, 3): 9,
    }

    #iterate through the blocks roi
    for i in blocks_roi.keys():
        blk = blocks_roi[i].copy()
        blk = cv2.cvtColor(blk,cv2.COLOR_BGR2GRAY)
        max_list = []

        #for eack blk rotate is by 90 degrees and try template matching
        for k in range(0,360,90):
            #cv2.resize(blk,(int(width/3),int(height/3)), interpolation= cv2.INTER_CUBIC)
            blk_copy = imutils.rotate_bound(blk, -k)

            #get the resulting heat map of template matching
            result = cv2.matchTemplate(jig_sol,blk_copy,cv2.TM_CCOEFF_NORMED)

            #get the max value and its location in the heat map
            _, max_val, _, max_loc = cv2.minMaxLoc(result)

            #append a tuple consisting of max location, value and peice rotation to max_list
            max_list.append((max_loc, max_val, k))#((k+1)*90)%360))

        #find the location with maximum value of template matching regardless of peice rotation
        top_left = max(max_list, key=lambda x: x[1])[0]

        #get the peice rotation of that template matching
        rot = max(max_list, key=lambda x: x[1])[2]

        #calculate the bottom right cordinates of the block
        bottom_right = (top_left[0] + int(width/3), top_left[1] + int(height/3))

        #find the center of the block
        centx = 0
        centy = 0
        for (l,m) in [top_left, bottom_right]:
            centx += l
            centy += m
        centx = int(centx/2)
        centy = int(centy/2)

        #get the puzzle peice block position in solution image
        piece = (math.ceil(3*centx/width), math.ceil(3*centy/height))

        if piece not in identity.keys():
            continue

        match_data.update({i: (identity[piece], rot)})

    return match_data
