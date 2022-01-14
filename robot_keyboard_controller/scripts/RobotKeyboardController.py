#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin
import numpy as np

import rospy
from tf import transformations
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState

class RobotKeyboardController(object):
    def __init__(self):
        self.robot_name = None
        self.robot_idx = None

        self.euler_angle = [0, 0, 0] # RPY

        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        return

    def loadRobotName(self, robot_name):
        self.robot_name = robot_name
        return True

    def setRobotIdx(self, robot_idx):
        self.robot_idx = robot_idx
        return True

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

    def getFaceDirection(self):
        x_axis_direction = np.array([1, 0, 0])

        rotation_matrix = self.getRotationMatrixFromEulerAngle(self.euler_angle)

        face_direction = np.dot(rotation_matrix, x_axis_direction)
        print("face_direction = ", face_direction)
        return face_direction

    def getLeftDirection(self):
        face_direction = self.getFaceDirection()
        face_direction_2d = [face_direction[0], face_direction[1]]
        left_direction = [-face_direction_2d[1], face_direction_2d[0]]
        print("left_direction = ", left_direction)
        return left_direction

    def getRightDirection(self):
        face_direction = self.getFaceDirection()
        face_direction_2d = [face_direction[0], face_direction[1]]
        right_direction = [-face_direction_2d[1], face_direction_2d[0]]
        print("right_direction = ", right_direction)
        return right_direction

    def getBackDirection(self):
        face_direction = self.getFaceDirection()
        back_direction = -face_direction
        print("back_direction = ", back_direction)
        return back_direction

    def getRobotState(self, robot_name):
        robot_state = self.get_model_state_proxy(robot_name, "")
        return robot_state

    def setRobotState(self, robot_name, position, orientation):
        robot_state = ModelState()
        robot_state.model_name = robot_name
        robot_state.pose.position.x = position[0]
        robot_state.pose.position.y = position[1]
        robot_state.pose.position.z = position[2]
        robot_state.pose.orientation.x = orientation[0]
        robot_state.pose.orientation.y = orientation[1]
        robot_state.pose.orientation.z = orientation[2]
        robot_state.pose.orientation.w = orientation[3]
        set_state = self.set_model_state_proxy(robot_state)
        return set_state.success

if __name__ == "__main__":
    robot_keyboard_controller = RobotKeyboardController()
    robot_keyboard_controller.loadRobotName("kinect_camera_")
    robot_keyboard_controller.setRobotIdx(0)
    robot_keyboard_controller.getRobotState("kinect_camera_0")
    robot_keyboard_controller.setRobotState("kinect_camera_0", [1, 1, 1], [0, 0, 0, 1])

