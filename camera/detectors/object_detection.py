from typing import Any, Optional
import cv2
import os
import numpy as np
from copy import deepcopy
from ultralytics import YOLO
from ultralytics.utils import ops
from detectors.detectors import BallDetection, BallDetector
import time

import logging

logger = logging.getLogger(__name__)


class ObjectDetector(BallDetector):
    def __init__(self, model_path):
        logger.info("initializing yolo: started")
        self.model = YOLO(model_path)
        logger.info("initializing yolo: finished")

        self.class_colour = {"Balls": (64, 255, 255)}

    def detect(
        self,
        frame: cv2.typing.MatLike,
        debug_frame: Optional[cv2.typing.MatLike] = None,
    ):
        """Detect tennis balls using yolo

        Args:
            frame (cv2.typing.MatLike): _description_
            debug_frame (Optional[cv2.typing.MatLike], optional): _description_. Defaults to None.

        Returns tuple:
            ball_coords: list of detections in format (np.array(x,y),radius)
            bboxes: list of detections in format (class,tensor(x,y,w,h))
        """
        bboxes = self.get_bboxes(frame)
        # logger.debug("found bboxes: %s", bboxes)
        img_out = deepcopy(frame)

        ball_coords = []

        for bbox in bboxes:
            x, y, w, h = bbox[1]
            radius = (w + h) / 4
            center = (
                x,
                y,
            )  # NOTE: from testing, yolo seems to give these as center instead of top-left as expected
            ball_coords.append((center, radius))

            # draw bounding box
            if debug_frame is not None:
                x1, y1, x2, y2 = np.asarray(ops.xywh2xyxy(bbox[1])).astype(np.int64)
                cv2.rectangle(
                    debug_frame,
                    (x1, y1),
                    (x2, y2),
                    self.class_colour[bbox[0]],
                    thickness=2,
                )

                # draw class label
                cv2.putText(
                    debug_frame,
                    bbox[0],
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    self.class_colour[bbox[0]],
                    2,
                )

        return (
            ball_coords,
            bboxes,
        )

    def get_bboxes(self, cv_img):
        """Find tennis ball bounding boxes using yolo

        Args:
            cv_img (_type_): _description_

        Returns:
            bboxes: list of detections in form (class,tensor(x,y,w,h))
        """
        preds = self.model.predict(cv_img, imgsz=320, verbose=False)

        # get bounding box and class label for target(s) detected
        bboxes = []
        coords = []
        for p in preds:
            boxes = p.boxes
            if boxes is None:
                return []
            for box in boxes:
                # bounding format in [x, y, width, height]
                b_cord = box.xywh[0]

                box_label = box.cls  # class label of the box

                bboxes.append([p.names[int(box_label)], np.asarray(b_cord)])

        return bboxes

    # testing
