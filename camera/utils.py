import cv2
import numpy as np

cam = cv2.VideoCapture(2)


def get_frame():
    ret, image = cam.read()
    return image

def save_frame(fname):
    frame = get_frame()
    cv2.imwrite(fname, frame)
    print('image saved...')

def release():
    cam.release()
    cv2.destroyAllWindows()

def isOpened():
    return cam.isOpened()
