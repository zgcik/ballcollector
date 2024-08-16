import cv2

cam = cv2.VideoCapture(0)


def get_frame():
    ret, image = cam.read()
    return image
