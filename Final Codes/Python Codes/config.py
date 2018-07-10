
#port to which the camera is connected
CAMERA_PORT = '/dev/video0'

#usb port to which the xbee is connected
USB_PORT = '/dev/ttyUSB0'

#image of arena configuration
ARENA_IMAGE = 'arena1.png'

#jigsaw solution image
JIGSAW_SOLUTION = 'jigsaw_3.jpg'

#size of the arena image
ARENA_SIZE = (640, 480)

#parameters for safety region size
CIRCULAR_SAFETY_FACTOR = 1.2
RECTANGULAR_SAFTEY_FACTOR = 3.8

#minimum distance between robots to detect as collision
COLLISION_DISTANCE = 160

#distance to be matianed from the block while picking and dropping
DISTANCE_FROM_BLOCK = 68

#size of blocks
BLOCK_SIZE = (38, 38)

#resize factor for obstacle map for faster computations
RESIZE_SCALE = 1/3

#size of robot
ROBOT_SIZE = (int(80*RESIZE_SCALE), int(80*RESIZE_SCALE))

#Robot 1 marker id
ROBOT_1_MARKER_ID = 8
#Robot 1 xbee address
ROBOT_1_XBee_ADDR = '\x00\x01'

#Robot 2 marker id
ROBOT_2_MARKER_ID = 11
#Robot 2 xbee address
ROBOT_2_XBee_ADDR = '\x00\x03'
