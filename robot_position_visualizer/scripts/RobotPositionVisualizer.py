#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin
import cv2
import numpy as np
import open3d as o3d

import rospy
from tf import transformations
from gazebo_msgs.srv import GetModelState

class RobotKeyboardController(object):
    def __init__(self):
        self.robot_name = None
        self.robot_num = None

        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        return

    def loadRobot(self, robot_name, robot_num):
        self.robot_name = robot_name
        self.robot_num = robot_num
        return True

    def getRobotState(self, robot_name):
        robot_state = self.get_model_state_proxy(robot_name, "")
        return robot_state

    def getEulerAngleFromQuaternion(self, quaternion):
        (roll, pitch, yaw) = transformations.euler_from_quaternion([
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]])

        return np.array([roll, pitch, yaw])

    def getQuaternionFromEulerAngle(self, euler_angle):
        quaternion = transformations.quaternion_from_euler(
            euler_angle[0], euler_angle[1], euler_angle[2])
        return np.array([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])

    def getRotationMatrixFromEulerAngle(self, euler_angle):
        R_x = np.array([
            [1, 0, 0],
            [0, cos(euler_angle[0]), -sin(euler_angle[0])],
            [0, sin(euler_angle[0]), cos(euler_angle[0])]
        ])
                    
        R_y = np.array([
            [cos(euler_angle[1]), 0, sin(euler_angle[1])],
            [0, 1, 0],
            [-sin(euler_angle[1]), 0, cos(euler_angle[1])]
        ])
                    
        R_z = np.array([
            [cos(euler_angle[2]), -sin(euler_angle[2]), 0],
            [sin(euler_angle[2]), cos(euler_angle[2]), 0],
            [0, 0, 1]
        ])
                    
        rotation_matrix = np.dot(R_z, np.dot( R_y, R_x ))
        return rotation_matrix

    def getForwardDirection(self, robot_state):
        x_axis_direction = np.array([1, 0, 0])

        robot_orientation = robot_state.pose.orientation

        robot_quaternion = [
            robot_orientation.x,
            robot_orientation.y,
            robot_orientation.z,
            robot_orientation.w]

        euler_angle = self.getEulerAngleFromQuaternion(robot_quaternion)

        rotation_matrix = self.getRotationMatrixFromEulerAngle(euler_angle)

        forward_direction = np.dot(rotation_matrix, x_axis_direction)
        forward_direction = np.array([forward_direction[0], forward_direction[1], 0])
        forward_direction_norm = np.linalg.norm(forward_direction)

        if forward_direction_norm == 0:
            print("RobotKeyboardController::getForwardDirection :")
            print("forward_direction_norm is 0!")
            return None

        forward_direction /= forward_direction_norm
        return forward_direction

    def getAllRobotState(self):
        robot_state_list = []

        if self.robot_name is None:
            print("RobotPositionVisualizer::getAllRobotState :")
            print("robot_name is None!")
            return None

        if self.robot_num is None:
            print("RobotPositionVisualizer::getAllRobotState :")
            print("robot_num is None!")
            return None

        if self.robot_num < 1:
            print("RobotPositionVisualizer::getAllRobotState :")
            print("robot_num not valid!")
            return None

        for robot_idx in range(self.robot_num):
            current_robot_full_name = self.robot_name + str(robot_idx)
            current_robot_state = self.getRobotState(current_robot_full_name)
            if current_robot_state is None:
                print("RobotPositionVisualizer::getAllRobotState :")
                print("getRobotState for " + current_robot_full_name + " failed!")
                return None
            robot_state_list.append(current_robot_state)
        return robot_state_list

    def showPosition(self):
        x_bound_min = -8
        x_bound_max = 8
        y_bound_min = -10
        y_bound_max = 7
        z_bound_min = -1
        z_bound_max = 5
        robot_color = [255, 0, 0]
        bound_color = [0, 255, 0]

        bound_point_list = [
            [x_bound_min, y_bound_min, z_bound_min],
            [x_bound_min, y_bound_min, z_bound_max],
            [x_bound_min, y_bound_max, z_bound_min],
            [x_bound_min, y_bound_max, z_bound_max],
            [x_bound_max, y_bound_min, z_bound_min],
            [x_bound_max, y_bound_min, z_bound_max],
            [x_bound_max, y_bound_max, z_bound_min],
            [x_bound_max, y_bound_max, z_bound_max]
        ]

        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[0, 0, 0])
        point_cloud = o3d.geometry.PointCloud()

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="RobotPositionVisualizer")
        render_option = vis.get_render_option()
        render_option.background_color = np.array([0, 0, 0])
        render_option.point_size = 50
        vis.add_geometry(axis_pcd)
        vis.add_geometry(point_cloud)

        while True:
            if ord('q') == cv2.waitKey(100):
                break

            robot_state_list = self.getAllRobotState()
            if robot_state_list is None:
                print("RobotPositionVisualizer::showPosition :")
                print("getAllRobotState failed!")
                return False

            points = []
            colors = []
            for bound_point in bound_point_list:
                points.append(bound_point)
                colors.append(bound_color)

            for robot_state in robot_state_list:
                points.append([
                    robot_state.pose.position.x,
                    robot_state.pose.position.y,
                    robot_state.pose.position.z
                ])
                colors.append(robot_color)

            point_cloud.points = o3d.utility.Vector3dVector(np.array(points))
            point_cloud.colors = o3d.utility.Vector3dVector(np.array(colors) / 255.0)

            vis.update_geometry(point_cloud)
            vis.poll_events()
            vis.update_renderer()
        return True

if __name__ == "__main__":
    robot_name = "kinect_camera_"
    robot_num = 1

    robot_keyboard_controller = RobotKeyboardController()
    robot_keyboard_controller.loadRobot(robot_name, robot_num)
    robot_keyboard_controller.showPosition()

