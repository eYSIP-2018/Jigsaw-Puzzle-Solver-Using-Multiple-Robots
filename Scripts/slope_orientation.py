import numpy as np
import math
global z1
def slope(px,py,tx,ty):
    ax=tx-px
    ay=ty-py
    try:
        z=ay/ax
        
        if abs(z)<1:
            z2=math.atan(1/z)
            z1=int(90-math.degrees(z2))
        else:
            z2=math.atan(z)
            z1=int(math.degrees(z2))
        
    except:
        if ay==0:
            z1=0
        if ax==0:
            if ay>0:
                z1=-90
            else:
                z1=90
                    
    if z1<0:
        z1=z1+180
    if ay<0:
        z1=z1+180
               
    z1=360-z1
    return z1


