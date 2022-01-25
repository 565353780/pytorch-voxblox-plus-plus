#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo

class VPPCVTest(object):
    def __init__(self):
        self.cap = None

        self.depth_camera_info_pub = rospy.Publisher("camera/depth/camera_info", CameraInfo, queue_size=1)
        self.depth_image_raw_pub = rospy.Publisher("camera/depth/image_raw", Image, queue_size=1)
        self.rgb_camera_info_pub = rospy.Publisher("camera/rgb/camera_info", CameraInfo, queue_size=1)
        self.rgb_image_raw_pub = rospy.Publisher("camera/rgb/image_raw", Image, queue_size=1)
        return

    def loadVideo(self, video_path):
        if not os.path.exists(video_path):
            print("VPPCVTest::loadVideo :")
            print("video :", video_path, "not exist!")
            return False
        self.cap = cv2.VideoCapture(video_path)
        return True

    def getFrame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("VPPCVTest::getFrame :")
            print("cap frame failed!")
            return None
        return frame

    def testFrame(self):
        frame = self.getFrame()
        if frame is None:
            return False
        print("start pub data process...")
        print("need to finish this later!")
        return True

    def startTest(self):
        fps = self.cap.get(cv2.CV_CAP_PROP_FPS)
        if fps is None or fps < 1:
            print("VPPCVTest::test :")
            print("cap frame failed!")
            fps = 1
        rate = rospy.Rate(fps)
        while self.testFrame():
            rate.sleep()
        return True

if __name__ == "__main__":
    rospy.init_node("VPPCVTest")
    video_path = rospy.get_param("/video_path")

    vpp_cv_test = VPPCVTest()
    vpp_cv_test.loadVideo(video_path)
    vpp_cv_test.startTest()

