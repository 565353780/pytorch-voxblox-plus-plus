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

point_move_dict = {
    "01": [-9.2, 0.1],
}

class PointCloudDiff(object):
    def __init__(self):
        self.scene_pointcloud_folder_path = None

        # dataset data
        self.scene_pointcloud = None
        self.scene_point_num = None

        self.object_pointcloud_list = None
        self.merge_object_pointcloud = None

        self.valid_object_pointcloud_list = None
        self.merge_valid_object_pointcloud_list = None

        # recon data
        self.object_pointcloud_save_path = None

        self.object_last_create_time = None
        self.object_last_modify_time = None

        self.log_start_time = None
        self.last_log_time = None

        sleep(10)
        self.get_map_proxy = rospy.ServiceProxy("/gsm_node/get_map", GetMap)
        self.tf_logger_proxy = rospy.ServiceProxy('/tensorboard_logger/log_scalar', ScalarToBool)
        return

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

    def updateObjectPointCloudSavePath(self):
        object_pointcloud_save_basepath = os.path.expanduser('~') + \
            "/.ros/RUN_LOG/PointCloud2ToObjectVecConverterServer/"
        pointcloud_save_folder_list = os.listdir(object_pointcloud_save_basepath)

        if len(pointcloud_save_folder_list) == 0:
            print("[ERROR][PointCloudDiff::getObjectPointCloudSavePath]")
            print("\t pointcloud_save_folder not exist!")
            return False

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
            print("[ERROR][PointCloudDiff::getObjectPointCloudSavePath]")
            print("\t find latest folder failed!")
            return False

        self.object_pointcloud_save_path = object_pointcloud_save_basepath + \
            max_idx_folder_name + "/"
        return True

    def getMergePointCloud(self, pointcloud_list):
        merge_pointcloud = o3d.geometry.PointCloud()
        points_list = []
        colors_list = []
        for pointcloud in pointcloud_list:
            points_list.append(np.array(pointcloud.points))
            colors_list.append(np.array(pointcloud.colors))
        merge_points = np.concatenate(points_list, axis=0)
        merge_colors = np.concatenate(colors_list, axis=0)
        merge_pointcloud.points = o3d.utility.Vector3dVector(merge_points)
        merge_pointcloud.colors = o3d.utility.Vector3dVector(merge_colors)
        return merge_pointcloud

    def loadObjectPointCloud(self):
        self.object_pointcloud_list = []

        scene_idx = self.scene_pointcloud_folder_path.split("/")[-2]
        scene_point_move_list = point_move_dict[scene_idx]

        object_pointcloud_folder_path = self.scene_pointcloud_folder_path + \
            "region_objects/"
        object_pointcloud_filename_list = os.listdir(object_pointcloud_folder_path)
        for object_pointcloud_filename in object_pointcloud_filename_list:
            object_pointcloud_filepath = object_pointcloud_folder_path + \
                object_pointcloud_filename
            object_pointcloud = o3d.io.read_point_cloud(object_pointcloud_filepath)
            object_points = np.array(object_pointcloud.points)
            object_points[:, :2] += scene_point_move_list
            object_pointcloud.points = \
                o3d.utility.Vector3dVector(object_points)
            self.object_pointcloud_list.append(object_pointcloud)

        self.merge_object_pointcloud = \
            self.getMergePointCloud(self.object_pointcloud_list)

        self.merge_object_pointcloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))
        return True

    def loadValidObjectPointCloud(self):
        self.valid_object_pointcloud_list = []

        scene_idx = self.scene_pointcloud_folder_path.split("/")[-2]
        scene_point_move_list = point_move_dict[scene_idx]

        object_pointcloud_folder_path = \
            self.scene_pointcloud_folder_path + "valid_region_objects/"
        object_pointcloud_filename_list = \
            os.listdir(object_pointcloud_folder_path)
        for object_pointcloud_filename in object_pointcloud_filename_list:
            object_pointcloud_filepath = object_pointcloud_folder_path + \
                object_pointcloud_filename
            object_pointcloud = o3d.io.read_point_cloud(object_pointcloud_filepath)
            object_points = np.array(object_pointcloud.points)
            object_points[:, :2] += scene_point_move_list
            object_pointcloud.points = \
                o3d.utility.Vector3dVector(object_points)
            self.valid_object_pointcloud_list.append(object_pointcloud)

        self.merge_valid_object_pointcloud = \
            self.getMergePointCloud(self.valid_object_pointcloud_list)

        self.merge_valid_object_pointcloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))
        return True

    def loadAllPointCloud(self, scene_pointcloud_folder_path):
        if not self.updateObjectPointCloudSavePath():
            print("[ERROR][PointCloudDiff::loadAllPointCloud]")
            print("\t updateObjectPointCloudSavePath failed!")
            return False
        if not self.loadScenePointCloud(scene_pointcloud_folder_path):
            print("[ERROR][PointCloudDiff::loadAllPointCloud]")
            print("\t loadScenePointCloud failed!")
            return False
        if not self.loadObjectPointCloud():
            print("[ERROR][PointCloudDiff::loadAllPointCloud]")
            print("\t loadObjectPointCloud failed!")
            return False
        if not self.loadValidObjectPointCloud():
            print("[ERROR][PointCloudDiff::loadAllPointCloud]")
            print("\t loadValidObjectPointCloud failed!")
            return False
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
        avg_dist2_error = 0
        for dist in dist_to_scene:
            avg_dist2_error += dist * dist
        avg_dist2_error /= dist_to_scene.shape[0]

        dist_to_recon = \
            self.scene_pointcloud.compute_point_cloud_distance(current_pcd)
        dist_to_recon = np.asarray(dist_to_recon)
        recon_point_num = len(np.where(dist_to_recon < 0.2)[0])
        recon_percent = 1.0 * recon_point_num / self.scene_point_num

        if not self.logScalar("PointCloudDiff/scene_error",
                              self.last_log_time - self.log_start_time,
                              avg_dist2_error):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for point_distance_mean failed!")
            return False
        if not self.logScalar("PointCloudDiff/scene_completeness",
                              self.last_log_time - self.log_start_time,
                              recon_percent):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for recon_percent failed!")
            return False
        return True

    def logObjectData(self):
        pointcloud_save_filename_list = \
            os.listdir(self.object_pointcloud_save_path)
        if len(pointcloud_save_filename_list) == 0:
            return True

        object_filename_list = []
        for pointcloud_save_filename in pointcloud_save_filename_list:
            if pointcloud_save_filename[:7] != "object_":
                continue
            object_filename_list.append(pointcloud_save_filename)

        if len(object_filename_list) == 0:
            return True

        object_current_create_time = os.path.getctime(
            self.object_pointcloud_save_path + "object_0.pcd")
        object_current_modify_time = os.path.getmtime(
            self.object_pointcloud_save_path + "object_0.pcd")
        if object_current_create_time == self.object_last_create_time and \
                object_current_modify_time == self.object_last_modify_time:
            return True

        self.object_last_create_time = object_current_create_time
        self.object_last_modify_time = object_current_modify_time

        recon_object_pointcloud_list = []
        for object_filename in object_filename_list:
            recon_object_pointcloud = o3d.io.read_point_cloud(
                self.object_pointcloud_save_path + object_filename)
            if np.array(recon_object_pointcloud.points).shape[0] == 0:
                continue
            recon_object_pointcloud_list.append(recon_object_pointcloud)

        recon_merge_object_pointcloud = self.getMergePointCloud(
            recon_object_pointcloud_list)
        recon_merge_object_pointcloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))

        dist_to_scene = \
            recon_merge_object_pointcloud.compute_point_cloud_distance(
                self.merge_object_pointcloud)
        dist_to_scene = np.asarray(dist_to_scene)
        avg_dist2_error = 0
        for dist in dist_to_scene:
            avg_dist2_error += dist * dist
        avg_dist2_error /= dist_to_scene.shape[0]

        dist_to_recon = \
            self.merge_object_pointcloud.compute_point_cloud_distance(
                recon_merge_object_pointcloud)
        dist_to_recon = np.asarray(dist_to_recon)
        recon_point_num = len(np.where(dist_to_recon < 0.2)[0])
        recon_percent = 1.0 * recon_point_num / self.scene_point_num

        if not self.logScalar("PointCloudDiff/object_error",
                              self.last_log_time - self.log_start_time,
                              avg_dist2_error):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for point_distance_mean failed!")
            return False
        if not self.logScalar("PointCloudDiff/object_completeness",
                              self.last_log_time - self.log_start_time,
                              recon_percent):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for recon_percent failed!")
            return False

        dist_to_scene = \
            recon_merge_object_pointcloud.compute_point_cloud_distance(
                self.merge_valid_object_pointcloud)
        dist_to_scene = np.asarray(dist_to_scene)
        avg_dist2_error = 0
        for dist in dist_to_scene:
            avg_dist2_error += dist * dist
        avg_dist2_error /= dist_to_scene.shape[0]

        dist_to_recon = \
            self.merge_valid_object_pointcloud.compute_point_cloud_distance(
                recon_merge_object_pointcloud)
        dist_to_recon = np.asarray(dist_to_recon)
        recon_point_num = len(np.where(dist_to_recon < 0.2)[0])
        recon_percent = 1.0 * recon_point_num / self.scene_point_num

        if not self.logScalar("PointCloudDiff/valid_object_error",
                              self.last_log_time - self.log_start_time,
                              avg_dist2_error):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for point_distance_mean failed!")
            return False
        if not self.logScalar("PointCloudDiff/valid_object_completeness",
                              self.last_log_time - self.log_start_time,
                              recon_percent):
            print("[ERROR][PointCloudDiff::startComparePointCloud]")
            print("\t logScalar for recon_percent failed!")
            return False
        return True

    def startComparePointCloud(self):
        self.log_start_time = time()
        self.last_log_time = self.log_start_time

        while True:
            sleep(10)

            new_log_time = time()
            if new_log_time == self.last_log_time:
                return True
            self.last_log_time = new_log_time

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
    pointcloud_diff.loadAllPointCloud(scene_pointcloud_folder_path)
    pointcloud_diff.startComparePointCloud()

