#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import torch
from detectron2.config import get_cfg
from detectron2.engine.defaults import DefaultPredictor

class Detectron2Detector(object):
    def __init__(self):
        self.model_path = None
        self.config_file = None
        self.confidence_threshold = 0.5

        self.cfg = None
        self.predictor = None
        return

    def loadModel(self,
                  model_path,
                  config_file):
        self.model_path = model_path
        self.config_file = config_file

        print("start loading model...", end="")
        self.cfg = get_cfg()
        self.cfg.merge_from_file(config_file)
        self.cfg.MODEL.WEIGHTS = model_path
        self.cfg.MODEL.RETINANET.SCORE_THRESH_TEST = self.confidence_threshold
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = self.confidence_threshold
        self.cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = self.confidence_threshold
        self.cfg.freeze()
        self.predictor = DefaultPredictor(self.cfg)
        print("SUCCESS!")
        return True

    def detect_image(self, image):
        """
        Args:
            image (np.ndarray): an image of shape (H, W, C) (in BGR order).
                This is the format used by OpenCV.

        Returns:
            predictions (dict): the output of the model.
        """
        result = self.predictor(image)

        pred_boxes = result["instances"].pred_boxes.tensor.cpu().numpy()
        scores = result["instances"].scores.cpu().numpy()
        pred_classes = result["instances"].pred_classes.cpu().numpy()
        pred_masks = result["instances"].pred_masks.cpu().numpy()

        result_dict = {}
        result_dict["pred_boxes"] = pred_boxes
        result_dict["scores"] = scores
        result_dict["pred_classes"] = pred_classes
        result_dict["pred_masks"] = pred_masks
        return result_dict

if __name__ == "__main__":
    model_path = "/home/chli/model_final_a3ec72.pkl"
    config_file = "../configs/COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x.yaml"

    detectron2_detector = Detectron2Detector()
    detectron2_detector.loadModel(model_path, config_file)

    image_path = "/home/chli/vpp_ws/test.jpg"
    image = cv2.imread(image_path)
    result_dict = detectron2_detector.detect_image(image)
    print(result_dict)

    for box in result_dict["pred_boxes"].astype(int):
        cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 3)
    cv2.imshow("result", image)
    cv2.waitKey(5000)

