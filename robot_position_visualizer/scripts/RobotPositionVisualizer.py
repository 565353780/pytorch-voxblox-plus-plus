#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin
import cv2
import numpy as np

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
        image_width = 1600
        image_height = 900
        robot_color = (0, 0, 255)

        while True:
            if ord('q') == cv2.waitKey(0):
                break

            robot_state_list = self.getAllRobotState()
            if robot_state_list is None:
                print("RobotPositionVisualizer::showPosition :")
                print("getAllRobotState failed!")
                return False

            robot_position_list = []
            for robot_state in robot_state_list:
                robot_position_list.append(robot_state.pose.position)

            x_min = robot_position_list[0].x
            x_max = x_min
            y_min = robot_position_list[0].y
            y_max = y_min
            z_min = robot_position_list[0].z
            z_max = z_min
            for robot_position in robot_position_list:
                x_min = min(x_min, robot_position.x)
                x_max = max(x_max, robot_position.x)
                y_min = min(y_min, robot_position.y)
                y_max = max(y_max, robot_position.y)
                z_min = min(z_min, robot_position.z)
                z_max = max(z_max, robot_position.z)

            x_center = (x_min + x_max) / 2.0
            y_center = (y_min + y_max) / 2.0
            z_center = (z_min + z_max) / 2.0
            center = [x_center, y_center, z_center]

            image = np.zeros((image_width, image_height, 3), np.uint8)
            image.fill(255)

            cv2.imshow("RobotPositionVisualizer", image)
        return True

if __name__ == "__main__":
    robot_name = "kinect_camera_"
    robot_num = 3

    robot_keyboard_controller = RobotKeyboardController()
    robot_keyboard_controller.loadRobot(robot_name, robot_num)
    robot_keyboard_controller.showPosition()

