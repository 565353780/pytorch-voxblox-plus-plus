#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import open3d as o3d

scene_pointcloud_file_path = \
    "/home/chli/.ros/COSCAN/MatterPort/01_ARNzJeq3xxb/matterport_01_1000000.ply"
object_pointcloud_folder_path = \
    "/home/chli/.ros/COSCAN/MatterPort/01_ARNzJeq3xxb/region_segmentations/"

pointcloud = o3d.io.read_point_cloud()

