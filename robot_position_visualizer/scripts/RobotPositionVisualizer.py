#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin, pi
import numpy as np

import rospy
from tf import transformations
from gazebo_msgs.msg import ModelState
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

    def getBackDirection(self, robot_state):
        forward_direction = self.getForwardDirection(robot_state)
        if forward_direction is None:
            print("RobotKeyboardController::getBackDirection :")
            print("forward_direction is None!")
            return None
        back_direction = -forward_direction
        return back_direction

    def getLeftDirection(self, robot_state):
        forward_direction = self.getForwardDirection(robot_state)
        if forward_direction is None:
            print("RobotKeyboardController::getLeftDirection :")
            print("forward_direction is None!")
            return None
        left_direction = [-forward_direction[1], forward_direction[0], forward_direction[2]]
        return left_direction

    def getRightDirection(self, robot_state):
        forward_direction = self.getForwardDirection(robot_state)
        if forward_direction is None:
            print("RobotKeyboardController::getRightDirection :")
            print("forward_direction is None!")
            return None
        right_direction = [forward_direction[1], -forward_direction[0], forward_direction[2]]
        return right_direction

    def getUpDirection(self, robot_state):
        up_direction = np.array([0, 0, 1])
        return up_direction

    def getDownDirection(self, robot_state):
        down_direction = np.array([0, 0, -1])
        return down_direction

    def showPosition(self):
        return True

if __name__ == "__main__":
    robot_name = "kinect_camera_"
    robot_num = 3

    robot_keyboard_controller = RobotKeyboardController()
    robot_keyboard_controller.loadRobot(robot_name, robot_num)
    robot_keyboard_controller.showPosition()

