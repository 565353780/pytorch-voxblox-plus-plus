#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import json
import open3d as o3d

scene_pointcloud_folder_path = "/home/chli/.ros/COSCAN/MatterPort/01/"

matterport_foldername = None
scene_pointcloud_folder_filename_list = os.listdir(scene_pointcloud_folder_path)
print(scene_pointcloud_folder_filename_list)
for scene_pointcloud_folder_filename in scene_pointcloud_folder_filename_list:
    if os.path.isfile(scene_pointcloud_folder_filename):
        continue
    matterport_foldername = scene_pointcloud_folder_filename
    break

scene_pointcloud_file_path = scene_pointcloud_folder_path + \
    matterport_foldername + "/house_segmentations/" + \
    matterport_foldername + ".ply"
object_pointcloud_folder_path = os.path.expanduser('~') + \
    matterport_foldername + "/region_segmentations/"

pointcloud = o3d.io.read_point_cloud(scene_pointcloud_file_path)

o3d.visualization.draw_geometries([pointcloud])

