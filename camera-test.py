from copy import deepcopy
import time
import numpy as np
import cv2
from camera.camera import Camera, imshow, is_headless
from camera.detectors.circle_detection import CircleDetector
from camera.detectors.line_detection import LineDetector
from camera.detectors.object_detection import ObjectDetector
from collections import deque
import imutils
import os
import camera.target_est as target_est


import logging

logger = logging.getLogger(__name__)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger.info("headless mode: %s", is_headless())
    cam = Camera()
    last = int(time.time())
    while True:
        cam.get_frame()

        # balls = cam.ball_detector((0, 0, 0))
        balls = cam.find_balls()

        if cam.debug_frame is not None:
            cv2.circle(cam.debug_frame, (320, 240), 2, (0, 255, 0), 2)
            imshow("debug", cam.debug_frame)

        if last < int(time.time()):
            logger.debug("detections: %s", balls)
            last = int(time.time())
