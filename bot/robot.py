import numpy as np
import cv2
import camera
import camera.camera

class Robot:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.cam = camera.camera.Camera()

    def detect(self, line=False, ball=False):
        self.cam.line_detector()
        self.cam.circle_detector()
    
    def drive(self):
        pass

    def set_vel_left_wheel(self, vel):
        pass

    def set_vel_right_wheel(self, vel):
        pass

    