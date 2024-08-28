from typing import Optional
import cv2

from collections import deque
import numpy as np
import imutils
import time

from camera.detectors.parameter_calibration import (
    HsvRangeCalibrator,
    ParameterCalibrator,
)

import logging

logger = logging.getLogger(__name__)


class CircleDetector:
    # Detection bounds
    max_count: int

    def __init__(self, int_matrix, max_count=2, calibrate=True) -> None:
        self.range = HsvRangeCalibrator(
            "circle", calibrate, (29, 23, 69), (50, 255, 255)
        )
        self.z = 0.0342
        self.int_matrix = int_matrix
        self.max_count = max_count
        self.open_kernel_size = ParameterCalibrator("contour", calibrate, 17, "open")
        self.close_kernel_size = ParameterCalibrator("contour", calibrate, 29, "close")

    def detect(
        self,
        frame: cv2.typing.MatLike,
        debug_frame: Optional[cv2.typing.MatLike] = None,
    ):
        """
        takes corrected bgr image
        """
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green"
        mask = cv2.inRange(hsv, self.range.lower.value(), self.range.upper.value())
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.open_kernel_size.value, self.open_kernel_size.value)
        )
        morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.close_kernel_size.value, self.close_kernel_size.value)
        )
        morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel)

        if debug_frame is not None:
            cv2.imshow("mask", morph)
        # find contours in the mask and initialize the current (x, y) center of the ball
        contours = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        center = None

        balls = []

        if len(contours) == 0:
            return []

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for i in range(min(self.max_count, len(contours))):
            c = contours[i]
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] != 0 and radius > 10:
                center = (float(M["m10"] / M["m00"]), float(M["m01"] / M["m00"]))
                center_i = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                z_est = (self.z * self.int_matrix[0][0]) / radius
                # drawing circles
                if debug_frame is not None:
                    cv2.circle(
                        debug_frame, (int(x), int(y)), int(radius), (0, 255, 255), 2
                    )
                    cv2.circle(debug_frame, center_i, 5, (0, 0, 255), -1)

                    # estimating depth
                    cv2.putText(
                        debug_frame,
                        f"d: {z_est:.2f}",
                        (int(x) - 50, int(y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

                # saving world coordinates
                balls.append((center, radius))

        # logger.debug("found circles: %s", balls)
        return balls
