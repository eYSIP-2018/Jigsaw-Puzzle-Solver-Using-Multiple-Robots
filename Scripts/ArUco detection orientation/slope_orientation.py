# import numpy as np
import math
global z1


# slope between two points
# px, py is the point whose slope need to be found
# tx, ty is the target point along which slope need to be found
def slope(px, py, tx, ty):
    ax = tx-px
    ay = ty-py
    try:
        z = ay/ax
        if abs(z) < 1:
            z2 = math.atan(1/z)
            z1 = int(90-math.degrees(z2))
        else:
            z2 = math.atan(z)
            z1 = int(math.degrees(z2))
    except Exception as e:
        # if slope is zero then angle is zero
        if ay == 0:
            z1 = 0
        # if slope is infinite
        if ax == 0:
            # if slope is positive infinite angle is -90 degrees
            if ay > 0:
                z1 = -90
            # if slope is negative infinite angle is 90 degrees
            else:
                z1 = 90
    # for negative angles and negative slopes angle is added with 180 degrees
    if z1 < 0:
        z1 = z1+180
    if ay < 0:
        z1 = z1+180
    # to get the angle from lower horizon
    z1 = 360-z1
    return z1
