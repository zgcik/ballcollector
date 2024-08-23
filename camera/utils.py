import cv2
import numpy as np

cam = cv2.VideoCapture(0)


def get_frame():
    ret, image = cam.read()
    return image


def release():
    cam.release()
    cv2.destroyAllWindows()


def isOpened():
    return cam.isOpened()
