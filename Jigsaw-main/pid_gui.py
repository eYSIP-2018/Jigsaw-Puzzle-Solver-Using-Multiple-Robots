import cv2

class PID:

    def __init__(self, robot_id):

        self.ROBOT_ID = robot_id

        cv2.namedWindow(self.ROBOT_ID)

        cv2.createTrackbar('kp', self.ROBOT_ID, 0, 500, lambda x: None)
        cv2.createTrackbar('ki', self.ROBOT_ID, 0, 500, lambda x: None)
        cv2.createTrackbar('kd', self.ROBOT_ID, 0, 500, lambda x: None)

        self.kp = cv2.getTrackbarPos('kp', self.ROBOT_ID)
        self.ki = cv2.getTrackbarPos('ki', self.ROBOT_ID)
        self.kd = cv2.getTrackbarPos('kd', self.ROBOT_ID)

    def set_parameters(self):
        self.kp = cv2.getTrackbarPos('kp', self.ROBOT_ID)
        self.ki = cv2.getTrackbarPos('ki', self.ROBOT_ID)
        self.kd = cv2.getTrackbarPos('kd', self.ROBOT_ID)

    def get_parameters(self):
        return self.kp, self.ki, self.kd