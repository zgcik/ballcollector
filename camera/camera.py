import numpy as np
import cv2
import utils
from collections import deque
import imutils
import os


class Camera:
    frame: cv2.typing.MatLike
    frame_gray: cv2.typing.MatLike
    buffer: int

    def __init__(self):
        # self.frame = np.ndarray([])
        # self.frame_gray = np.ndarray([])
        script_dir = os.path.dirname(__file__)
        self.int_matrix = np.load(
            os.path.join(script_dir, "calibration", "int_matrix.npy")
        )
        self.dist_matrix = np.load(
            os.path.join(script_dir, "calibration", "dist_matrix.npy")
        )

        self.green_lower = (29, 86, 6)
        self.green_upper = (64, 255, 255)
        self.buffer = 5

        # all in m
        self.objects = {"ball": 0.0342}

    def get_frame(self):
        frame = utils.get_frame()
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

    def line_detector(self):
        # canny edge detection
        edges = cv2.Canny(self.frame_gray, 50, 150, apertureSize=3)

        # hough line transform
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10
        )

        if lines is not None:
            lines = np.squeeze(lines)
            if lines.ndim == 2 and lines.shape[1] == 4:
                cx, cy = self.int_matrix[0, 2], self.int_matrix[1, 2]
                distances = {}

                for line in lines:
                    x1, y1, x2, y2 = line
                    cv2.line(self.frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # convert line endpoints to normalized camera coordinates
                    p1_norm = np.array(
                        [
                            (x1 - cx) / self.int_matrix[0, 0],
                            (y1 - cy) / self.int_matrix[1, 1],
                        ]
                    )
                    p2_norm = np.array(
                        [
                            (x2 - cx) / self.int_matrix[0, 0],
                            (y2 - cy) / self.int_matrix[1, 1],
                        ]
                    )

                return distances

        else:
            return None

    def circle_detector(self, max_count=1):
        pts = deque(maxlen=max_count)

        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green"
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)  # type: ignore
        mask = cv2.erode(mask, None, iterations=2)  # type: ignore
        mask = cv2.dilate(mask, None, iterations=2)  # type: ignore

        # find contours in the mask and initialize the current (x, y) center of the ball
        contours = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = imutils.grab_contours(contours)
        hist = cv2.equalizeHist(self.frame_gray)
        blur = cv2.GaussianBlur(hist, (63, 63), cv2.BORDER_DEFAULT)
        cv2.imshow("blur", blur)
        cv2.waitKey(1)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)
        height, width = blur.shape

        minR = round(width / 20)
        maxR = round(width / 3)
        minDis = round(width / 7)
        circles = cv2.HoughCircles(
            blur,
            cv2.HOUGH_GRADIENT,
            1,
            10,
            param1=30,
            param2=50,
            minRadius=minR,
            maxRadius=maxR,
        )
        # print(circles)
        if circles is not None:

            circles = np.around(circles).astype(np.uint16)
            for i in circles[0, :]:
                # draw the outer circle
                cv2.circle(self.frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle
                cv2.circle(self.frame, (i[0], i[1]), 2, (0, 0, 255), 3)

        center = None

        balls = []

        if len(contours) == 0:
            return

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for i in range(min(max_count, len(contours))):
            c = contours[i]
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] != 0 and radius > 10:
                center = (float(M["m10"] / M["m00"]), float(M["m01"] / M["m00"]))
                center_i = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # drawing circles
                cv2.circle(self.frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(self.frame, center_i, 5, (0, 0, 255), -1)

                # estimating depth
                z_est = (self.objects.get("ball") * self.int_matrix[0][0]) / radius
                cv2.putText(
                    self.frame,
                    f"d: {z_est:.2f}",
                    (int(x) - 50, int(y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

                # saving world coordinates
                w_coords = [self.pixel_to_camera(center) * z_est, z_est]
                balls.append(w_coords)

            pts.appendleft(center)

        # for i in range(1, len(pts)):
        #     if pts[i - 1] is None or pts[i] is None:
        #         continue
        #     thickness = int(np.sqrt(self.buffer / float(i + 1)) * 2.5)
        #     cv2.line(self.frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

        return balls


if __name__ == "__main__":
    cam = Camera()
    while True:
        cam.get_frame()
        # line detection
        # line_dis = cam.line_detector()

        # circle detection
        circles = cam.circle_detector(max_count=2)

        if cv2.waitKey(1) == ord("q"):
            break

        cv2.imshow("camera feed", cam.frame)
