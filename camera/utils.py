import cv2

cam = cv2.VideoCapture(2)


def get_frame():
    ret, image = cam.read()
    return image

def release():
    cam.release()
    cv2.destroyAllWindows()

def isOpened():
    return cam.isOpened()
