from copy import deepcopy
import numpy as np
import cv2
from detectors.circle_detection import CircleDetector
from detectors.line_detection import LineDetector
from detectors.object_detection import ObjectDetector
from collections import deque
import imutils
import os
import camera.target_est as target_est


import logging

logger = logging.getLogger(__name__)

try:
    cv2.waitKey(1)
    headless = False
except:
    headless = True
logger.info("headless mode:", headless)


def is_headless():
    return headless


def imshow(name: str, frame: np.ndarray):
    if not headless:
        cv2.imshow(name, frame)
        cv2.waitKey(1)


class Camera:
    frame: cv2.typing.MatLike
    frame_gray: cv2.typing.MatLike
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
        self.circle_detector = CircleDetector(self.int_matrix)
        self.object_detector = ObjectDetector(
            os.path.join(os.path.dirname(__file__), "detectors/model.pt")
        )
        self.line_detector = LineDetector(self.int_matrix)
        self.targets = []

    def get_frame(self):
        _, frame = self.camera.read()
        if frame is None:
            return
        self.frame = self.undistort_img(frame)
        self.frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

    def undistort_img(self, frame):
        h, w = frame.shape[:2]
        n_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.int_matrix, self.dist_matrix, (w, h), 1, (w, h)
        )
        undistorted = cv2.undistort(
            frame, self.int_matrix, self.dist_matrix, None, n_camera_matrix
        )
        return undistorted

    def pixel_to_camera(self, pix_coords):
        x = (pix_coords[0] - self.int_matrix[0, 2]) / self.int_matrix[0, 0]
        y = (pix_coords[1] - self.int_matrix[1, 2]) / self.int_matrix[1, 1]

        return np.array([x, y])

    def find_balls(self):
        if is_headless():
            debug_frame = None
        else:
            debug_frame = deepcopy(self.frame)
        balls = self.circle_detector.detect(self.frame, debug_frame)
        if len(balls) == 0:
            balls = self.object_detector.detect(self.frame, debug_frame)
        if debug_frame is not None:
            imshow("debug", debug_frame)
        return balls

    def ball_detector(self, rob_pose):
        bboxes, _ = self.object_detector.detect(self.frame)
        for detection in bboxes:
            self.targets.append(
                target_est.target_pose_est(self.int_matrix, detection, rob_pose)
            )

        if len(bboxes) > 0:
            self.detections = target_est.merge_ests(self.detections)

        return self.detections

    def find_lines(self):
        lines = self.line_detector.detect(self.frame_gray)
        return lines


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    cam = Camera()
    while True:
        cam.get_frame()
        # line detection
        # line_dis = cam.line_detector()  # TODO: requires calibration of parameters

        # circle detection
        circles = cam.find_balls()
