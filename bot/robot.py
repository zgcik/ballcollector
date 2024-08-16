import numpy as np
import cv2

class Robot:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port

    def set_vel_left_wheel(self, vel):
        pass

    def set_vel_right_wheel(self, vel):
        pass

    def cam_capture(self, cam):
        cv2.capture