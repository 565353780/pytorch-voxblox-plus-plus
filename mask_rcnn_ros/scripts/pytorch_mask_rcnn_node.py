#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import threading
import numpy as np
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest

from mask_rcnn_ros import visualize
from mask_rcnn_ros.msg import Result

from Detectron2Detector import Detectron2Detector

RGB_TOPIC = '/camera/rgb/image_raw'

# COCO Class names
# Index of the class in the list is its ID. For example, to get ID of
# the teddy bear class, use: CLASS_NAMES.index('teddy bear')
CLASS_NAMES = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
               'bus', 'train', 'truck', 'boat', 'traffic light',
               'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
               'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
               'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
               'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
               'kite', 'baseball bat', 'baseball glove', 'skateboard',
               'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
               'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
               'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
               'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
               'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
               'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
               'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
               'teddy bear', 'hair drier', 'toothbrush']

class MaskRCNNNode(object):
    def __init__(self):
        self._cv_bridge = CvBridge()

        # Get input RGB topic.
        self._rgb_input_topic = rospy.get_param('~input', RGB_TOPIC)

        self._visualization = rospy.get_param('~visualization', True)

        self._class_names = rospy.get_param('~class_names', CLASS_NAMES)

        self._last_msg = None
        self._msg_lock = threading.Lock()

        self._class_colors = visualize.random_colors(len(CLASS_NAMES))

        self._publish_rate = rospy.get_param('~publish_rate', 100)

        model_path = os.environ['HOME'] + "/.ros/model_final_a3ec72.pkl"
        config_file = os.environ['HOME'] + "/.ros/configs/COCO-InstanceSegmentation/mask_rcnn_R_101_FPN_3x.yaml"
        self.detector = Detectron2Detector()
        self.detector.loadModel(model_path, config_file)
        return

    def run(self):
        self._result_pub = rospy.Publisher('~result', Result, queue_size=1)
        vis_pub = rospy.Publisher('~visualization', Image, queue_size=1)

        rospy.Subscriber(self._rgb_input_topic, Image,
                         self._image_callback, queue_size=1)

        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue

            if msg is not None:
                np_image = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

                # Run detection
                result_dict = self.detector.detect_image(np_image)

                result = {}
                rois = result_dict["pred_boxes"].astype(int)
                trans_rois = []
                for box in rois:
                    trans_rois.append([box[1], box[0], box[3], box[2]])
                result["rois"] = np.array(trans_rois)
                result["scores"] = result_dict["scores"]
                result["class_ids"] = result_dict["pred_classes"]
                result["masks"] = result_dict["pred_masks"].astype(np.uint8).transpose(1, 2, 0)

                result_msg = self._build_result_msg(msg, result)
                self._result_pub.publish(result_msg)

                # Visualize results
                if self._visualization:
                    cv_result = self._visualize_plt(result, np_image)
                    image_msg = self._cv_bridge.cv2_to_imgmsg(cv_result, 'bgr8')
                    vis_pub.publish(image_msg)

            rate.sleep()

    def _build_result_msg(self, msg, result):
        result_msg = Result()
        result_msg.header = msg.header
        for i, (y1, x1, y2, x2) in enumerate(result['rois']):
            box = RegionOfInterest()
            box.x_offset = x1
            box.y_offset = y1
            box.height = y2 - y1
            box.width = x2 - x1
            result_msg.boxes.append(box)

            class_id = result['class_ids'][i]
            result_msg.class_ids.append(class_id)

            class_name = self._class_names[class_id]
            result_msg.class_names.append(class_name)

            score = result['scores'][i]
            result_msg.scores.append(score)

            mask = Image()
            mask.header = msg.header
            mask.height = result['masks'].shape[0]
            mask.width = result['masks'].shape[1]
            mask.encoding = "mono8"
            mask.is_bigendian = False
            mask.step = mask.width
            mask.data = (result['masks'][:, :, i] * 255).tobytes()
            result_msg.masks.append(mask)
        return result_msg

    def _visualize(self, result, image):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        fig = Figure()
        canvas = FigureCanvasAgg(fig)
        axes = fig.gca()
        visualize.display_instances(image, result['rois'], result['masks'],
                                    result['class_ids'], CLASS_NAMES,
                                    result['scores'], ax=axes,
                                    class_colors=self._class_colors)
        fig.tight_layout()
        canvas.draw()
        result = np.fromstring(canvas.tostring_rgb(), dtype='uint8')

        _, _, w, h = fig.bbox.bounds
        result = result.reshape((int(h), int(w), 3))
        return result

    def _get_fig_ax(self):
        """Return a Matplotlib Axes array to be used in
        all visualizations. Provide a
        central point to control graph sizes.

        Change the default size attribute to control the size
        of rendered images
        """
        fig, ax = plt.subplots(1)
        plt.subplots_adjust(
            left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
        return fig, ax

    def _visualize_plt(self, result, image):
        fig, ax = self._get_fig_ax()
        image = visualize.display_instances_plt(
            image,
            result['rois'],
            result['masks'],
            result['class_ids'],
            CLASS_NAMES,
            result['scores'],
            fig=fig,
            ax=ax)

        return image

    def _image_callback(self, msg):
        rospy.logdebug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

def main():
    rospy.init_node('mask_rcnn')

    node = MaskRCNNNode()
    node.run()

if __name__ == "__main__":
    main()

