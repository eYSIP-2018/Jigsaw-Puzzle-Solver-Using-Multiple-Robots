import numpy as np

from config import *

def distance(a, b):
    """Returns the distance between two diferent points
    Returns:
        (int): distance between the two points
    """
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def remap(i, from_size, to_size):
    """Remaps the pixel cordinate from from_size image to to_size image
    Arguments:
        i ((int, int)): point to be remapped
        from_size ((int, int)): size of source image
        to_size ((int, int)): size of destination image
    Returns:
        ((int, int)): remapped point
    """
    return (int(i[0]*to_size[0]/from_size[0]), int(i[1]*to_size[1]/from_size[1]))

def downscale(i, scale=RESIZE_SCALE):
    """Downscales the given point by given scale
    Arguments:
        i ((int, int)): point to be downscaled
        scale ((int, int)): scale to which the point should be downscaled
    Returns:
        ((int, int)): downscaled point
    """
    return (int(i[0]*scale), int(i[1]*scale))

def upscale(i, scale=RESIZE_SCALE):
    """Downscales the given point by given scale
    Arguments:
        i ((int, int)): point to be upscaled
        scale ((int, int)): scale to which the point should be upscaled
    Returns:
        ((int, int)): upscaled point
    """
    return (int(i[0]/scale), int(i[1]/scale))

def nearest_point(pt, pts_list):
    """Of list of points returns the point nearest to the given point
    Arguments:
        pt ((int, int)): given point
        pts_list (list): list of points
    """
    dist = 100000000
    nearest = None
    for i in pts_list:
        if dist > distance(pt, i[:2]):
            dist = distance(pt, i[:2])
            nearest = i
    return nearest

def get_adjacents(pt, blocks):
    """Returns a list containing adjacent points around the given point
    Arguments:
        pt ((int, int)): cordinates of the point
    Returns:
        adjacent (list): a list containing adjacent points
    """
    adjacents = []
    adjacents.append((pt[0]+DISTANCE_FROM_BLOCK, pt[1], 270))
    adjacents.append((pt[0]-DISTANCE_FROM_BLOCK, pt[1], 90))
    adjacents.append((pt[0], pt[1]+DISTANCE_FROM_BLOCK, 0))
    adjacents.append((pt[0], pt[1]-DISTANCE_FROM_BLOCK, 180))
    
    for i in adjacents[:]:
        try:
            if blocks[i[1], i[0]] <127:
                adjacents.remove(i)
        except:
            adjacents.remove(i)

    return adjacents

def get_nearest_obstacle(blocks, obstacles, i):
    """Returns the obstacle nearest to the given point
    Arguments:
        obstacles (dict): a dictionary containing the position of the obstacles as keys and width, height of obstacles as values
    Returns:
        ((int, int), (int, int)): positon of neareast obstacle, nearest adjacent point of the obstacle
    """
    near_obstacle = nearest_point(i, obstacles.keys())
    return near_obstacle, nearest_point(i, get_adjacents(near_obstacle, blocks))

def search_by_peice_num(match_data, peice_num):
    """Returns the position of given peice number
    Arguments:
        match_data (dict): a dictionary containing keys as position of peices with values as peice number
        peice_num (int): query peice number
    Returns:
        i ((int, int)): peice position
    """
    for i in match_data.keys():
        num, _ = match_data[i]
        if num == peice_num:
            return i

def get_grid(size, blocksize=BLOCK_SIZE[0], padding=20):
    """Returns a dictionary containing the keys as peice number with value as cordinates where the block needs to be placed
    Arguments:
        size ((int, int)): size of arena
        blocksize (int): size of block
        padding (int): padding between blocks
    Returns:
        grid (dict): a dictionary containing the keys as peice number with value as cordinates where the block needs to be placed
    """
    h, w = size
    totalsize = blocksize + padding

    grid = {
        5: (int(h/2), int(w/2)),
        8: (int(h/2), int(w/2 + totalsize)),
        2: (int(h/2), int(w/2 - totalsize)),
        4: (int(h/2 - totalsize), int(w/2)),
        6: (int(h/2 + totalsize), int(w/2)),
        1: (int(h/2 - totalsize), int(w/2 - totalsize)),
        3: (int(h/2 + totalsize), int(w/2 - totalsize)),
        7: (int(h/2 - totalsize), int(w/2 + totalsize)),
        9: (int(h/2 + totalsize), int(w/2 + totalsize)),
    }

    return grid