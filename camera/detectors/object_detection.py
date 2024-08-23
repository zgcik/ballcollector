from typing import Optional
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
    ) -> list[BallDetection]:
        bboxes = self.get_bboxes(frame)
        logger.debug("found bboxes: %s", bboxes)
        img_out = deepcopy(frame)

        balls = []

        for bbox in bboxes:
            xyxy = ops.xywh2xyxy(bbox[1])
            x1 = int(xyxy[0])
            y1 = int(xyxy[1])
            x2 = int(xyxy[2])
            y2 = int(xyxy[3])
            radius = (x2 - x1 + y2 - y1) / 2
            center = ((x2 - x1) / 2, (y2 - y1) / 2)
            balls.append({"center": center, "radius": radius})

            # draw bounding box
            if debug_frame is not None:
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
        return balls

    def get_bboxes(self, cv_img):
        preds = self.model.predict(cv_img, imgsz=320, verbose=False)

        # get bounding box and class label for target(s) detected
        bboxes = []
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