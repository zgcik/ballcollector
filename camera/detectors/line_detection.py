from typing import Optional
import cv2

from collections import deque
import numpy as np

import logging

logger = logging.getLogger(__name__)


class LineDetector:
    def __init__(self, int_matrix) -> None:
        self.int_matrix = int_matrix

    def detect(
        self,
        frame_gray: cv2.typing.MatLike,
        debug_frame: Optional[cv2.typing.MatLike] = None,
    ):
        # canny edge detection
        edges = cv2.Canny(frame_gray, 50, 150, apertureSize=3)

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
                    if debug_frame is not None:
                        cv2.line(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

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
