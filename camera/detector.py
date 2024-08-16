import cv2
import numpy as np
import os

from copy import deepcopy
from ultralytics import YOLO
from ultralytics.utils import ops

class Detector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
    
    def _get_bounding_boxes(self, img):
        preds = self.model.predict(img)
        
        bboxes = []
        for p in preds:
            boxes = preds.boxes

            for b in boxes:
                b_coords = b.xywh
                b_label = b.cls

                bboxes.append([preds.names[int(b_label)], np.asarray(b_coords)])

        return bboxes
    
    def detect_img(self, img):
        bboxes = self._get_bounding_boxes(img)
        img_out = deepcopy(img)
        for bb in bboxes:
            xyxy = ops.xywh2xyxy(bb[1])
            x1 = int(xyxy[0])
            y1 = int(xyxy[1])
            x2 = int(xyxy[2])
            y2 = int(xyxy[3])

            img_out = cv2.rectangle(img_out, bb[0], (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        return bboxes, img_out
