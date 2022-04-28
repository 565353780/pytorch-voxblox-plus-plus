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
        self.scene_pointcloud_folder_path = None
        self.scene_pointcloud = None
        self.scene_point_num = None

        self.total_pointcloud_save_path = None

        sleep(10)
        self.log_start_time = None
        self.last_log_time = None

        self.get_map_proxy = rospy.ServiceProxy("/gsm_node/get_map", GetMap)
        self.tf_logger_proxy = rospy.ServiceProxy('/tensorboard_logger/log_scalar', ScalarToBool)

        self.updatePointCloudSavePath()
        return

    def updatePointCloudSavePath(self):
        total_pointcloud_save_basepath = os.path.expanduser('~') + \
            "/.ros/RUN_LOG/PointCloud2ToObjectVecConverterServer/"
        pointcloud_save_folder_list = os.listdir(total_pointcloud_save_basepath)
        pointcloud_save_folder_list.sort()

        max_idx_list = None
        max_idx_folder_name = None
        for pointcloud_save_folder in pointcloud_save_folder_list:
            date_split_list = pointcloud_save_folder.split("_")
            time_split_list = date_split_list[3].split("-")
            current_idx_list = [
                int(date_split_list[0]),
                int(date_split_list[1]),
                int(date_split_list[2]),
                int(time_split_list[0]),
                int(time_split_list[1]),
                int(time_split_list[2])
            ]
            if max_idx_list is None:
                max_idx_list = current_idx_list
                max_idx_folder_name = pointcloud_save_folder
                continue

            for i in range(len(max_idx_list)):
                if current_idx_list[i] > max_idx_list[i]:
                    max_idx_list = current_idx_list
                    max_idx_folder_name = pointcloud_save_folder
                    break
                if current_idx_list[i] < max_idx_list[i]:
                    break

        if max_idx_folder_name is None:
            print("[ERROR][PointCloudDiff::updatePointCloudSavePath]")
            print("\t find latest folder failed!")
            return False

        self.total_pointcloud_save_path = \
            total_pointcloud_save_basepath + max_idx_folder_name
        return True

    def loadScenePointCloud(self, scene_pointcloud_folder_path):
        self.scene_pointcloud_folder_path = scene_pointcloud_folder_path
        if self.scene_pointcloud_folder_path[-1] != "/":
            self.scene_pointcloud_folder_path += "/"

        if not os.path.exists(self.scene_pointcloud_folder_path):
            print("[ERROR][PointCloudDiff::loadScenePointCloud]")
            print("\t scene_pointcloud_folder not exist!")
            return False

        scene_pointcloud_folder_filename_list = \
            os.listdir(self.scene_pointcloud_folder_path)
        scene_pointcloud_filename = None
        for scene_pointcloud_folder_filename in scene_pointcloud_folder_filename_list:
            if ".ply" not in scene_pointcloud_folder_filename:
                continue
            scene_pointcloud_filename = scene_pointcloud_folder_filename
            break

        scene_pointcloud_file_path = \
            self.scene_pointcloud_folder_path + scene_pointcloud_filename

        pointcloud_file_path_split_list = scene_pointcloud_file_path.split(".")
        if pointcloud_file_path_split_list[-1] == "obj":
            mesh = o3d.io.read_triangle_mesh(scene_pointcloud_file_path)
            self.scene_pointcloud = o3d.geometry.PointCloud()
            self.scene_pointcloud.points = o3d.utility.Vector3dVector(mesh.vertices)
            self.scene_point_num = len(self.scene_pointcloud.points)
            return True

        self.scene_pointcloud = o3d.io.read_point_cloud(scene_pointcloud_file_path)
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

    def logSceneData(self):
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
        if new_log_time == self.last_log_time:
            return True

        self.last_log_time = new_log_time
        if not self.logScalar("PointCloudDiff/point_distance_mean",
                              new_log_time - self.log_start_time,
                              current_mean):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for point_distance_mean failed!")
            return False
        if not self.logScalar("PointCloudDiff/point_distance_var",
                              new_log_time - self.log_start_time,
                              current_var):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for point_distance_var failed!")
            return False
        if not self.logScalar("PointCloudDiff/3d_recon_percent",
                              new_log_time - self.log_start_time,
                              current_3d_recon_percent):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for 3d_recon_percent failed!")
            return False
        return True

    def logObjectData(self):
        pointcloud_save_filename_list = os.listdir(self.total_pointcloud_save_path)
        if len(pointcloud_save_filename_list) == 0:
            return True

        object_filename_list = []
        for pointcloud_save_filename in pointcloud_save_filename_list:
            if pointcloud_save_filename[:7] != "object_":
                continue
            object_filename_list.append(pointcloud_save_filename)

        print(object_filename_list)
        return True

    def startComparePointCloud(self):
        self.log_start_time = time()
        self.last_log_time = self.log_start_time

        while True:
            sleep(10)

            if not self.logSceneData():
                print("[ERROR][PointCloudDiff::startComparePointCloud]")
                print("\t logSceneData failed!")
                break
            if not self.logObjectData():
                print("[ERROR][PointCloudDiff::startComparePointCloud]")
                print("\t logObjectData failed!")
                break
        return True

if __name__ == "__main__":
    rospy.init_node("PointCloudDiff")
    scene_pointcloud_folder_path = \
        os.path.expanduser('~') + "/" + \
        rospy.get_param("/scene_pointcloud_folder_path")

    pointcloud_diff = PointCloudDiff()
    pointcloud_diff.loadScenePointCloud(scene_pointcloud_folder_path)
    pointcloud_diff.startComparePointCloud()

