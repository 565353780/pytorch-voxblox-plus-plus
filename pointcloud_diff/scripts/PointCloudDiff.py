#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
import open3d as o3d
from time import sleep

import rospy
from sensor_msgs.point_cloud2 import read_points

from vpp_msgs.srv import GetMap
from tensorboard_logger_ros.msg import Scalar
from tensorboard_logger_ros.srv import ScalarToBool

class PointCloudDiff(object):
    def __init__(self):
        self.scene_pointcloud = None

        #  sleep(10)
        self.get_map_proxy = rospy.ServiceProxy("/gsm_node/get_map", GetMap)
        self.tf_logger_proxy = rospy.ServiceProxy('/tensorboard_logger/log_scalar', ScalarToBool)
        return

    def loadScenePointCloud(self, pointcloud_file_path):
        if not os.path.exists(pointcloud_file_path):
            print("[ERROR][PointCloudDiff]")
            print("\t pointcloud_file not exist!")
            return False

        pointcloud_file_path_split_list = pointcloud_file_path.split(".")
        if pointcloud_file_path_split_list[-1] == "obj":
            mesh = o3d.io.read_triangle_mesh(pointcloud_file_path)
            self.scene_pointcloud = o3d.geometry.PointCloud()
            self.scene_pointcloud.points = o3d.utility.Vector3dVector(mesh.vertices)
            return True

        self.scene_pointcloud = o3d.io.read_point_cloud(pointcloud_file_path)
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
        while True:
            sleep(10)
            pointcloud2_msg = self.get_map_proxy()
            current_pcd = self.loadPointCloud2Msg(pointcloud2_msg.map_cloud)
            current_pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
            o3d.visualization.draw_geometries([current_pcd, self.scene_pointcloud])
            break
        return True

if __name__ == "__main__":
    rospy.init_node("PointCloudDiff")
    pointcloud_file_path = rospy.get_param("/scene_pointcloud_file_path")

    pointcloud_diff = PointCloudDiff()
    pointcloud_diff.loadScenePointCloud(pointcloud_file_path)
    pointcloud_diff.startComparePointCloud()

