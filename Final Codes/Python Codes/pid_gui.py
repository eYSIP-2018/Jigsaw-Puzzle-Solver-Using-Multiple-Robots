import cv2

class PID:
    """A class to implement PID GUI for easy manual tunning
    
    Attributes:
        ROBOT_ID (str): id of robot on which the pid is to be implemented
        kp (int): proportional gain parameter
        ki (int): integral gain parameter
        kd (int): differential gain parameter

    Methods:
        set_parameters(): gets values from the trackbars and sets the respective gain values
        get_parameters(): returns the gain values
    """

    def __init__(self, robot_id):
        """Creates an instance of PID class
        Arguments:
            robot_id (str): id of robot on which the pid is to be implemented
        """

        self.ROBOT_ID = robot_id

        cv2.namedWindow(self.ROBOT_ID)

        cv2.createTrackbar('kp', self.ROBOT_ID, 0, 500, lambda x: None)
        cv2.createTrackbar('ki', self.ROBOT_ID, 0, 500, lambda x: None)
        cv2.createTrackbar('kd', self.ROBOT_ID, 0, 500, lambda x: None)

        self.kp = cv2.getTrackbarPos('kp', self.ROBOT_ID)
        self.ki = cv2.getTrackbarPos('ki', self.ROBOT_ID)
        self.kd = cv2.getTrackbarPos('kd', self.ROBOT_ID)

    def set_parameters(self):
        """Gets values from the trackbars and sets the respective gain values"""
        self.kp = cv2.getTrackbarPos('kp', self.ROBOT_ID)
        self.ki = cv2.getTrackbarPos('ki', self.ROBOT_ID)
        self.kd = cv2.getTrackbarPos('kd', self.ROBOT_ID)

    def get_parameters(self):
        """Returns the gain values
        Returns:
            (int, int, int): proportional gain, integral gain, derivative gain
        """
        return self.kp, self.ki, self.kd