import cv2

from collections import deque
import numpy as np
import utils

def detect_lines():

    if not utils.isOpened():
        print("error: could not open camera.")
        exit()

    while True:
        # retrieving image frame
        frame = utils.get_frame()

        # greyscaling img
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # canny edge detection
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        # hough line transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow('camera feed', frame)

        # break with 'q'
        if cv2.waitKey(1) == ord('q'):
            break
    
    

if __name__ == "__main__":
    # begin line detection
    detect_lines()

    # release capture
    utils.release()
