#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
import open3d as o3d
from time import time, sleep

import rospy
from sensor_msgs.point_cloud2 import read_points

from vpp_msgs.srv import GetMap
from tensorboard_logger_ros.msg import Scalar
from tensorboard_logger_ros.srv import ScalarToBool

class PointCloudDiff(object):
    def __init__(self):
        self.scene_pointcloud = None
        self.scene_point_num = None

        sleep(10)
        self.get_map_proxy = rospy.ServiceProxy("/gsm_node/get_map", GetMap)
        self.tf_logger_proxy = rospy.ServiceProxy('/tensorboard_logger/log_scalar', ScalarToBool)
        return

    def loadScenePointCloud(self, pointcloud_file_path):
        if not os.path.exists(pointcloud_file_path):
            print("[ERROR][PointCloudDiff::loadScenePointCloud]")
            print("\t scene_pointcloud_file not exist!")
            return False

        pointcloud_file_path_split_list = pointcloud_file_path.split(".")
        if pointcloud_file_path_split_list[-1] == "obj":
            mesh = o3d.io.read_triangle_mesh(pointcloud_file_path)
            self.scene_pointcloud = o3d.geometry.PointCloud()
            self.scene_pointcloud.points = o3d.utility.Vector3dVector(mesh.vertices)
            self.scene_point_num = len(self.scene_pointcloud.points)
            return True

        self.scene_pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)
        self.scene_point_num = len(self.scene_pointcloud.points)
        return True

    def logScalar(self, name, step, value):
        scalar = Scalar()
        scalar.name = str(name)
        scalar.step = int(step)
        scalar.value = float(value)
        log_success = self.tf_logger_proxy(scalar)
        return log_success

    def loadPointCloud2Msg(self, pointcloud2_msg):
        point_list = \
            read_points(pointcloud2_msg,
                        skip_nans=True,
                        field_names=("x", "y", "z"))

        point_array = []

        for point in point_list:
            point_array.append(point[0:3])

        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(np.array(point_array))
        return pointcloud

    def startComparePointCloud(self):
        log_start_time = time()
        last_log_time = log_start_time

        while True:
            sleep(10)

            pointcloud2_msg = self.get_map_proxy()
            current_pcd = self.loadPointCloud2Msg(pointcloud2_msg.map_cloud)

            dist_to_scene = current_pcd.compute_point_cloud_distance(self.scene_pointcloud)
            dist_to_scene = np.asarray(dist_to_scene)
            current_mean = dist_to_scene.mean()
            current_var = dist_to_scene.var()

            dist_to_recon = self.scene_pointcloud.compute_point_cloud_distance(current_pcd)
            dist_to_recon = np.asarray(dist_to_recon)
            current_recon_point_num = len(np.where(dist_to_recon < 0.2)[0])
            current_3d_recon_percent = 1.0 * current_recon_point_num / self.scene_point_num

            new_log_time = time()
            if new_log_time == last_log_time:
                continue

            last_log_time = new_log_time
            if not self.logScalar("PointCloudDiff/point_distance_mean",
                                  new_log_time - log_start_time,
                                  current_mean):
                print("[ERROR][PointCloudDiff::startComparePointCloud]")
                print("\t logScalar for point_distance_mean failed!")
                break
            if not self.logScalar("PointCloudDiff/point_distance_var",
                                  new_log_time - log_start_time,
                                  current_var):
                print("[ERROR][PointCloudDiff::startComparePointCloud]")
                print("\t logScalar for point_distance_var failed!")
                break
            if not self.logScalar("PointCloudDiff/3d_recon_percent",
                                  new_log_time - log_start_time,
                                  current_3d_recon_percent):
                print("[ERROR][PointCloudDiff::startComparePointCloud]")
                print("\t logScalar for 3d_recon_percent failed!")
                break
        return True

if __name__ == "__main__":
    rospy.init_node("PointCloudDiff")
    pointcloud_file_path = rospy.get_param("/scene_pointcloud_file_path")
    pointcloud_file_path = os.path.expanduser('~') + "/" + pointcloud_file_path
    print(pointcloud_file_path)
    exit()

    pointcloud_diff = PointCloudDiff()
    pointcloud_diff.loadScenePointCloud(pointcloud_file_path)
    pointcloud_diff.startComparePointCloud()

