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

    def showPosition2D(self):
        image_width = 1600
        image_height = 900
        free_area_width = 50
        x_min = -8
        x_max = 7
        y_min = -9
        y_max = 9
        point_size = 5
        robot_color = (0, 0, 255)
        boundary_color = (255, 255, 255)

        valid_width = image_width - 2 * free_area_width
        valid_height = image_height - 2 * free_area_width

        if valid_width < 1 or valid_height < 1:
            print("RobotPositionVisualizer::showPosition2D :")
            print("free_area_width out of range!")
            return False

        x_diff = x_max - x_min
        y_diff = y_max - y_min

        if x_diff <= 0 or y_diff <= 0:
            print("RobotPositionVisualizer::showPosition2D :")
            print("x or y interval not valid!")
            return False

        boundary_polygon_in_world = [
            [x_min, y_min],
            [x_min, y_max],
            [x_max, y_max],
            [x_max, y_min],
        ]

        half_width = image_width / 2.0
        half_height = image_height / 2.0
        x_center = (x_min + x_max) / 2.0
        y_center = (y_min + y_max) / 2.0

        scale = min(
            1.0 * valid_height / x_diff,
            1.0 * valid_width / y_diff)

        def getPointInImage(point_in_world):
            point_in_image = [
                int(free_area_width + scale * (point_in_world[0] - x_min)),
                int(free_area_width + scale * (point_in_world[1] - y_min))
            ]
            return point_in_image

        while True:
            robot_state_list = self.getAllRobotState()
            if robot_state_list is None:
                print("RobotPositionVisualizer::showPosition2D :")
                print("getAllRobotState failed!")
                return False

            image = np.zeros((image_height, image_width, 3), np.uint8)

            boundary_polygon_in_image = []
            for boundary_point_in_world in boundary_polygon_in_world:
                boundary_point_in_image = getPointInImage(boundary_point_in_world)
                boundary_polygon_in_image.append([boundary_point_in_image[1], boundary_point_in_image[0]])

            cv2.polylines(image, [np.array(boundary_polygon_in_image)], True, boundary_color)

            for robot_state in robot_state_list:
                robot_x_in_world = robot_state.pose.position.x
                robot_y_in_world = robot_state.pose.position.y

                robot_position_in_world = [robot_x_in_world, robot_y_in_world]
                robot_position_in_image = getPointInImage(robot_position_in_world)

                cv2.circle(image, (robot_position_in_image[1], robot_position_in_image[0]), point_size, robot_color, 4)

            cv2.imshow("RobotPositionVisualizer2D", image)

            if ord('q') == cv2.waitKey(100):
                break
        return True

    def showPosition3D(self):
        robot_color = [255, 0, 0]

        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=2.0, origin=[0, 0, 0])
        point_cloud = o3d.geometry.PointCloud()

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="RobotPositionVisualizer3D")
        render_option = vis.get_render_option()
        render_option.background_color = np.array([0, 0, 0])
        render_option.point_size = 50
        vis.add_geometry(axis_pcd)
        vis.add_geometry(point_cloud)

        while True:
            robot_state_list = self.getAllRobotState()
            if robot_state_list is None:
                print("RobotPositionVisualizer::showPosition3D :")
                print("getAllRobotState failed!")
                return False

            points = []
            colors = []

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

            if ord('q') == cv2.waitKey(100):
                break
        return True

if __name__ == "__main__":
    robot_name = "kinect_camera_"
    robot_num = 1

    robot_keyboard_controller = RobotKeyboardController()
    robot_keyboard_controller.loadRobot(robot_name, robot_num)
    robot_keyboard_controller.showPosition2D()
    #  robot_keyboard_controller.showPosition3D()

