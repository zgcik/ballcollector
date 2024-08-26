from typing import Optional
import cv2

from collections import deque
import numpy as np

import logging

from detectors.parameter_calibration import HsvRangeCalibrator, ParameterCalibrator

logger = logging.getLogger(__name__)


def draw_line(img: cv2.typing.MatLike, params: np.ndarray):
    vx = int(params.item(0))
    vy = int(params.item(1))
    x = int(params.item(2))
    y = int(params.item(3))
    mult = max(img.shape[0], img.shape[1])
    start = (x - mult * vx, y - mult * vy)
    end = (x + mult * vx, y + mult * vy)
    _, start, end = cv2.clipLine((0, 0, img.shape[0], img.shape[1]), start, end)
    cv2.line(img, start, end, (0, 255, 0), 2)


class LineDetector:
    def __init__(self, int_matrix, calibrate=True) -> None:
        self.int_matrix = int_matrix
        self.range = HsvRangeCalibrator("line", calibrate, (0, 0, 205), (179, 56, 255))
        self.open_kernel_size = ParameterCalibrator("contour", calibrate, 3, "open")
        self.close_kernel_size = ParameterCalibrator("contour", calibrate, 29, "close")
        self.contour_threshold = ParameterCalibrator(
            "contour", calibrate, 29, "threshold", max_val=5000
        )

    def detect(
        self,
        frame: cv2.typing.MatLike,
        frame_gray: cv2.typing.MatLike,
        frame_hsv: cv2.typing.MatLike,
        debug_frame: Optional[cv2.typing.MatLike] = None,
    ):
        # thresholding
        mask = cv2.inRange(
            frame_hsv, self.range.lower.value(), self.range.upper.value()
        )
        if debug_frame is not None:
            cv2.imshow("linemask", mask)

        # cleaning up noise
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.open_kernel_size.value, self.open_kernel_size.value)
        )
        morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.close_kernel_size.value, self.close_kernel_size.value)
        )
        morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel)

        # find contours
        contours = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        for c in contours:
            area = cv2.contourArea(c)
            if area > self.contour_threshold.value:
                line = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
                if debug_frame is not None:
                    cv2.drawContours(debug_frame, [c], -1, (0, 0, 255), 2)
                    draw_line(debug_frame, line)

        # # canny edge detection
        # edges = cv2.Canny(mask, 50, 150, apertureSize=3)

        # # hough line transform
        # lines = cv2.HoughLinesP(
        #     edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10
        # )

        # if lines is not None:
        #     lines = np.squeeze(lines)
        #     if lines.ndim == 2 and lines.shape[1] == 4:
        #         cx, cy = self.int_matrix[0, 2], self.int_matrix[1, 2]
        #         distances = {}

        #         for line in lines:
        #             x1, y1, x2, y2 = line
        #             if debug_frame is not None:
        #                 cv2.line(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        #             # convert line endpoints to normalized camera coordinates
        #             p1_norm = np.array(
        #                 [
        #                     (x1 - cx) / self.int_matrix[0, 0],
        #                     (y1 - cy) / self.int_matrix[1, 1],
        #                 ]
        #             )
        #             p2_norm = np.array(
        #                 [
        #                     (x2 - cx) / self.int_matrix[0, 0],
        #                     (y2 - cy) / self.int_matrix[1, 1],
        #                 ]
        #             )

        #         return distances

        # else:
        #     return None
