import cv2
import numpy as np
import camera.utils
from pi import Pi

def calc_dist(rob_pose, target):
    return np.linalg.norm(np.array(point2) - np.array(point1))

def run(disp=True):
    pi = Pi()

    try:
        while True:
            detections = pi.cam.ball_detector(pi.rob_pose)

            if detections is not None:
                # find closest
                distances = []
                for d in detections.values():
                    distances.append(calc_dist(pi.rob_pose[:2], d))
                min_dist, ind = np.min(distances)
                
                # calculate path

                # drive to point for each point in the path

                # drive back to start

