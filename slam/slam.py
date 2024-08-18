import numpy as np

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rtabmap_ros.srv import *

def load_calibration():
    # loading in intrinsic matrices...
    in_matrix = np.load('./camera/calibration/camera_matrix.npy')
    dcoeff_matrix = np.load('./camera/calibration/dist_coeffs.npy')
    return in_matrix, dcoeff_matrix

def slam_run():
    # loading calibration
    in_matrix, dcoeff_matrix = load_calibration()

    # running slam
    