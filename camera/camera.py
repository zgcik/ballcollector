from copy import deepcopy
import time
from typing import Optional
import numpy as np
import cv2
import torch
from detectors.circle_detection import CircleDetector
from detectors.line_detection import LineDetector
from detectors.object_detection import ObjectDetector
from collections import deque
import imutils
import os
import target_est


import logging

logger = logging.getLogger(__name__)

try:
    cv2.imshow("", np.ndarray([]))
    cv2.waitKey(1)
    headless = False
except cv2.error as e:
    headless = True


def is_headless():
    return headless


def imshow(name: str, frame: Optional[np.ndarray]):
    if not headless and frame is not None:
        cv2.imshow(name, frame)
        cv2.waitKey(1)


class Camera:
    frame: cv2.typing.MatLike
    frame_gray: cv2.typing.MatLike
    frame_hsv: cv2.typing.MatLike
    debug_frame: Optional[cv2.typing.MatLike] = None
    circle_detector: CircleDetector
    oject_detector: ObjectDetector
    line_detector: LineDetector

    def __init__(self, device=0):
        self.camera = cv2.VideoCapture(device)
        script_dir = os.path.dirname(__file__)
        self.int_matrix = np.load(
            os.path.join(script_dir, "calibration", "int_matrix.npy")
        )
        self.dist_matrix = np.load(
            os.path.join(script_dir, "calibration", "dist_matrix.npy")
        )
        self.circle_detector = CircleDetector(self.int_matrix, calibrate=False)
        self.object_detector = ObjectDetector(
            os.path.join(os.path.dirname(__file__), "detectors/model.pt")
        )
        self.line_detector = LineDetector(self.int_matrix, calibrate=(not headless))
        self.targets = []
        self.camera_dims = (640, 480)

    def get_frame(self):
        _, frame = self.camera.read()
        if frame is None:
            return
        self.frame = self.undistort_img(frame)
        self.frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.frame_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        if not headless:
            self.debug_frame = deepcopy(self.frame)

    def undistort_img(self, frame):
        h, w = frame.shape[:2]
        n_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.int_matrix, self.dist_matrix, (w, h), 1, (w, h)
        )
        undistorted = cv2.undistort(
            frame, self.int_matrix, self.dist_matrix, None, n_camera_matrix
        )
        return undistorted

    def pixel_to_camera(self, ball_info: tuple[tuple[float, float], float]):
        """Get (x,y,z) camera coordinates of center of ball

        Args:
            ball_info (tuple[tuple[float,float],float]): _description_

        Returns:
            np.ndarray(float): (x,y,z) of ball in camera coordinates
        """
        center = ball_info[0]
        x = (center[0] - self.int_matrix[0, 2]) / self.int_matrix[0, 0]
        y = (center[1] - self.int_matrix[1, 2]) / self.int_matrix[1, 1]
        return np.array([x, y, self.get_distance(ball_info[1])])

    def get_distance(self, radius, z=0.0342):
        z_est = (z * self.int_matrix[0][0]) / radius
        return z_est

    def shift_origin(self, coord):
        x, y = coord[0] - self.camera_dims[0] / 2, self.camera_dims[1] / 2 - coord[1]
        return (x, y)

    def find_balls(self):
        # balls = self.circle_detector.detect(self.frame, self.debug_frame)
        # if len(balls) == 0:
        balls, _ = self.object_detector.detect(self.frame, self.debug_frame)
        return balls

    def find_balls_camera_coords(self):
        balls, _ = self.object_detector.detect(self.frame, self.debug_frame)
        logger.debug("balls: %s", balls)
        return list(
            map(
                lambda x: (self.shift_origin(x[0]), x[1]),
                balls,
            )
        )

    def ball_detector(self, rob_pose):
        _, bboxes = self.object_detector.detect(self.frame)
        for detection in bboxes:
            self.targets.append(
                target_est.target_pose_est(self.int_matrix, detection, rob_pose)
            )

        if len(bboxes) > 0:
            self.detections = target_est.merge_ests(self.detections)

        return self.detections

    def find_lines(self):
        lines = self.line_detector.detect(
            self.frame, self.frame_gray, self.frame_hsv, self.debug_frame
        )
        return lines


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("headless mode: %s", headless)
    cam = Camera()
    while True:
        cam.get_frame()
        # circle detection
        # circles = cam.find_lines()

        ball_coords = cam.find_balls_camera_coords()

        if ball_coords:
            logger.debug("Camera coord ball coordinates: %s", ball_coords)
            logger.debug("frame size: %s", cam.frame.shape)
            cv2.circle(cam.debug_frame, (320, 240), 5, (255, 255, 0), 2)  # type: ignore

        imshow("debug", cam.debug_frame)
        # time.sleep(1)
