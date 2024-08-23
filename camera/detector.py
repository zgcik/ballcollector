import cv2
import os
import numpy as np
from copy import deepcopy
from ultralytics import YOLO
from ultralytics.utils import ops
import utils

class Detector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

        self.class_colour = {
            'Balls': (64, 255, 255)
        }

        
    def detect_img(self, img):
        bboxes = self.get_bboxes(img)
        img_out = deepcopy(img)

        for bbox in bboxes:
            xyxy = ops.xywh2xyxy(bbox[1])
            x1 = int(xyxy[0])
            y1 = int(xyxy[1])
            x2 = int(xyxy[2])
            y2 = int(xyxy[3])

            # draw bounding box
            img_out = cv2.rectangle(img_out, (x1, y1), (x2, y2), self.class_colour[bbox[0]], thickness=2)

            # draw class label
            img_out = cv2.putText(img_out, bbox[0], (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                  self.class_colour[bbox[0]], 2)
        return bboxes, img_out
    
    def get_bboxes(self, cv_img):
        preds = self.model.predict(cv_img, imgsz=320, verbose=False)

        # get bounding box and class label for target(s) detected
        bboxes = []
        for p in preds:
            boxes = p.boxes
            for box in boxes:
                # bounding format in [x, y, width, height]
                b_cord = box.xywh[0]

                box_label = box.cls  # class label of the box

                bboxes.append([p.names[int(box_label)], np.asarray(b_cord)])

        return bboxes
    
    # testing
    if __name__ == '__main__':
        script_dir = os.path.dirname(os.path.abspath(__file__))
        from detector import Detector
        yolo = Detector(f'{script_dir}/model.pt')

        while True:
            img = utils.get_frame()
            bboxes, img_out = yolo.detect_img(img)
            print(bboxes)
            print(len(bboxes))

            cv2.imshow('yolo detect', img_out)

            if cv2.waitKey(1) == ord('q'): break