#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import json
import numpy as np
import open3d as o3d
from time import time
from tqdm import tqdm
from plyfile import PlyData

COLOR_MAP = {
    0: (0, 0, 0),
    1: (174, 199, 232),
    2: (152, 223, 138),
    3: (31, 119, 180),
    4: (255, 187, 120),
    5: (188, 189, 34),
    6: (140, 86, 75),
    7: (255, 152, 150),
    8: (214, 39, 40),
    9: (197, 176, 213),
    10: (148, 103, 189),
    11: (196, 156, 148),
    12: (23, 190, 207),
    13: (100, 85, 144),
    14: (247, 182, 210),
    15: (66, 188, 102),
    16: (219, 219, 141),
    17: (140, 57, 197),
    18: (202, 185, 52),
    19: (51, 176, 203),
    20: (200, 54, 131),
    21: (92, 193, 61),
    22: (78, 71, 183),
    23: (172, 114, 82),
    24: (255, 127, 14),
    25: (91, 163, 138),
    26: (153, 98, 156),
    27: (140, 153, 101),
    28: (158, 218, 229),
    29: (100, 125, 154),
    30: (178, 127, 135),
    32: (146, 111, 194),
    33: (44, 160, 44),
    34: (112, 128, 144),
    35: (96, 207, 209),
    36: (227, 119, 194),
    37: (213, 92, 176),
    38: (94, 106, 211),
    39: (82, 84, 163),
    40: (100, 85, 144),
}

scene_pointcloud_folder_path = "/home/chli/.ros/COSCAN/MatterPort/01/"

matterport_foldername = None
scene_pointcloud_folder_filename_list = os.listdir(scene_pointcloud_folder_path)
for scene_pointcloud_folder_filename in scene_pointcloud_folder_filename_list:
    if os.path.isfile(scene_pointcloud_folder_filename):
        continue
    matterport_foldername = scene_pointcloud_folder_filename
    break

scene_pointcloud_file_path = scene_pointcloud_folder_path + \
    matterport_foldername + "/house_segmentations/" + \
    matterport_foldername + ".ply"
object_pointcloud_folder_path = scene_pointcloud_folder_path + \
    matterport_foldername + "/region_segmentations/"

#  pointcloud = o3d.io.read_point_cloud(scene_pointcloud_file_path)
#  o3d.visualization.draw_geometries([pointcloud])

test_file_path = object_pointcloud_folder_path + "region0.ply"
print("start loading ply file...")
start = time()
plydata = PlyData.read(test_file_path)
print("load finish in ", time() - start, " s!")

vertex_list = []
for vertex in tqdm(plydata['vertex']):
    vertex_list.append([vertex['x'], vertex['y'], vertex['z']])

vertex_category_list = [-1 for _ in vertex_list]
for face in tqdm(plydata['face']):
    vertex_index_list = face['vertex_indices']
    catrgory_id = face['category_id']
    for vertex_index in vertex_index_list:
        vertex_category_list[vertex_index] = catrgory_id

not_valid_vertex_num = 0
for vertex_category in vertex_category_list:
    if vertex_category == -1:
        not_valid_vertex_num += 1

print("not_valid_vertex_num = ", not_valid_vertex_num)

pcd = o3d.geometry.PointCloud()
vertex_array = np.array(vertex_list)
pcd.points = o3d.utility.Vector3dVector(vertex_array)
colors = [COLOR_MAP[i % 40] for i in list(vertex_category_list)]
pcd.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries([pcd])

