#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin, pi
import numpy as np
import getch

import rospy
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState

class RobotKeyboardController(object):
    def __init__(self):
        self.move_forward_key = "e"
        self.move_left_key = "s"
        self.move_right_key = "f"
        self.move_back_key = "d"
        self.move_up_key = "r"
        self.move_down_key = "w"

        self.turn_left_key = "j"
        self.turn_right_key = "l"
        self.look_up_key = "i"
        self.look_down_key = "k"
        self.head_left_key = "u"
        self.head_right_key = "o"

        self.move_dist = 0.1
        self.rotate_angle = 10

        self.move_key_list = [
            self.move_forward_key, self.move_back_key,
            self.move_left_key, self.move_right_key,
            self.move_up_key, self.move_down_key
        ]
        self.rotate_key_list = [
            self.turn_left_key, self.turn_right_key,
            self.look_up_key, self.look_down_key,
            self.head_left_key, self.head_right_key
        ]

        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        return

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

    def moveRobot(self, robot_name, robot_state, move_direction, move_dist):
        new_position = [
            robot_state.pose.position.x + move_dist * move_direction[0],
            robot_state.pose.position.y + move_dist * move_direction[1],
            robot_state.pose.position.z + move_dist * move_direction[2]
        ]

        robot_orientation = [
            robot_state.pose.orientation.x,
            robot_state.pose.orientation.y,
            robot_state.pose.orientation.z,
            robot_state.pose.orientation.w
        ]

        if not self.setRobotState(robot_name, new_position, robot_orientation):
            print("RobotKeyboardController::moveRobot :")
            print("setRobotState failed!")
            return False
        return True

    def rotateRobot(self, robot_name, robot_state, rotate_angle, rotate_axis):
        robot_position = [
            robot_state.pose.position.x,
            robot_state.pose.position.y,
            robot_state.pose.position.z
        ]

        robot_orientation = [
            robot_state.pose.orientation.x,
            robot_state.pose.orientation.y,
            robot_state.pose.orientation.z,
            robot_state.pose.orientation.w
        ]

        euler_angle = self.getEulerAngleFromQuaternion(robot_orientation)

        real_rotate_angle = rotate_angle * pi / 180.0

        if rotate_axis == 0:
            euler_angle[0] += real_rotate_angle
        elif rotate_axis == 1:
            euler_angle[1] += real_rotate_angle
        elif rotate_axis == 2:
            euler_angle[2] += real_rotate_angle
        else:
            print("RobotKeyboardController::rotateRobot :")
            print("rotate_axis out of range!")
            return False

        new_orientation = self.getQuaternionFromEulerAngle(euler_angle)

        if not self.setRobotState(robot_name, robot_position, new_orientation):
            print("RobotKeyboardController::rotateRobot :")
            print("setRobotState failed!")
            return False
        return True

    def moveForward(self, robot_name, move_dist):
        robot_state = self.getRobotState(robot_name)

        move_direction = self.getForwardDirection(robot_state)
        if move_direction is None:
            print("RobotKeyboardController::moveForward :")
            print("move_direction is None!")
            return False

        if not self.moveRobot(robot_name, robot_state, move_direction, move_dist):
            print("RobotKeyboardController::moveForward :")
            print("moveRobot failed!")
            return False
        return True

    def moveBack(self, robot_name, move_dist):
        robot_state = self.getRobotState(robot_name)

        move_direction = self.getBackDirection(robot_state)
        if move_direction is None:
            print("RobotKeyboardController::moveBack :")
            print("move_direction is None!")
            return False

        if not self.moveRobot(robot_name, robot_state, move_direction, move_dist):
            print("RobotKeyboardController::moveBack :")
            print("moveRobot failed!")
            return False
        return True

    def moveLeft(self, robot_name, move_dist):
        robot_state = self.getRobotState(robot_name)

        move_direction = self.getLeftDirection(robot_state)
        if move_direction is None:
            print("RobotKeyboardController::moveLeft :")
            print("move_direction is None!")
            return False

        if not self.moveRobot(robot_name, robot_state, move_direction, move_dist):
            print("RobotKeyboardController::moveLeft :")
            print("moveRobot failed!")
            return False
        return True

    def moveRight(self, robot_name, move_dist):
        robot_state = self.getRobotState(robot_name)

        move_direction = self.getRightDirection(robot_state)
        if move_direction is None:
            print("RobotKeyboardController::moveRight :")
            print("move_direction is None!")
            return False

        if not self.moveRobot(robot_name, robot_state, move_direction, move_dist):
            print("RobotKeyboardController::moveRight :")
            print("moveRobot failed!")
            return False
        return True

    def moveUp(self, robot_name, move_dist):
        robot_state = self.getRobotState(robot_name)

        move_direction = self.getUpDirection(robot_state)
        if move_direction is None:
            print("RobotKeyboardController::moveUp :")
            print("move_direction is None!")
            return False

        if not self.moveRobot(robot_name, robot_state, move_direction, move_dist):
            print("RobotKeyboardController::moveUp :")
            print("moveRobot failed!")
            return False
        return True

    def moveDown(self, robot_name, move_dist):
        robot_state = self.getRobotState(robot_name)

        move_direction = self.getDownDirection(robot_state)
        if move_direction is None:
            print("RobotKeyboardController::moveDown :")
            print("move_direction is None!")
            return False

        if not self.moveRobot(robot_name, robot_state, move_direction, move_dist):
            print("RobotKeyboardController::moveDown :")
            print("moveRobot failed!")
            return False
        return True

    def turnLeft(self, robot_name, rotate_angle):
        robot_state = self.getRobotState(robot_name)

        if not self.rotateRobot(robot_name, robot_state, rotate_angle, 2):
            print("RobotKeyboardController::turnLeft :")
            print("rotateRobot failed!")
            return False
        return True

    def turnRight(self, robot_name, rotate_angle):
        robot_state = self.getRobotState(robot_name)

        right_rotate_angle = -rotate_angle

        if not self.rotateRobot(robot_name, robot_state, right_rotate_angle, 2):
            print("RobotKeyboardController::turnRight :")
            print("rotateRobot failed!")
            return False
        return True

    def lookUp(self, robot_name, rotate_angle):
        robot_state = self.getRobotState(robot_name)

        up_rotate_angle = -rotate_angle

        if not self.rotateRobot(robot_name, robot_state, up_rotate_angle, 1):
            print("RobotKeyboardController::lookUp :")
            print("rotateRobot failed!")
            return False
        return True

    def lookDown(self, robot_name, rotate_angle):
        robot_state = self.getRobotState(robot_name)

        if not self.rotateRobot(robot_name, robot_state, rotate_angle, 1):
            print("RobotKeyboardController::lookDown :")
            print("rotateRobot failed!")
            return False
        return True

    def headLeft(self, robot_name, rotate_angle):
        robot_state = self.getRobotState(robot_name)

        left_rotate_angle = -rotate_angle

        if not self.rotateRobot(robot_name, robot_state, left_rotate_angle, 0):
            print("RobotKeyboardController::headRight :")
            print("rotateRobot failed!")
            return False
        return True

    def headRight(self, robot_name, rotate_angle):
        robot_state = self.getRobotState(robot_name)

        if not self.rotateRobot(robot_name, robot_state, rotate_angle, 0):
            print("RobotKeyboardController::headRight :")
            print("rotateRobot failed!")
            return False
        return True

    def keyBoardMove(self, robot_name, move_dist, keyboard_input):
        if keyboard_input == self.move_forward_key:
            return self.moveForward(robot_name, move_dist)
        if keyboard_input == self.move_back_key:
            return self.moveBack(robot_name, move_dist)
        if keyboard_input == self.move_left_key:
            return self.moveLeft(robot_name, move_dist)
        if keyboard_input == self.move_right_key:
            return self.moveRight(robot_name, move_dist)
        if keyboard_input == self.move_up_key:
            return self.moveUp(robot_name, move_dist)
        if keyboard_input == self.move_down_key:
            return self.moveDown(robot_name, move_dist)

        print("RobotKeyboardController::keyBoardMove :")
        print("keyboard_input out of range!")
        return False

    def keyBoardRotate(self, robot_name, rotate_angle, keyboard_input):
        if keyboard_input == self.turn_left_key:
            return self.turnLeft(robot_name, rotate_angle)
        if keyboard_input == self.turn_right_key:
            return self.turnRight(robot_name, rotate_angle)
        if keyboard_input == self.look_up_key:
            return self.lookUp(robot_name, rotate_angle)
        if keyboard_input == self.look_down_key:
            return self.lookDown(robot_name, rotate_angle)
        if keyboard_input == self.head_left_key:
            return self.headLeft(robot_name, rotate_angle)
        if keyboard_input == self.head_right_key:
            return self.headRight(robot_name, rotate_angle)

        print("RobotKeyboardController::keyBoardRotate :")
        print("keyboard_input out of range!")
        return False

    def keyBoardControl(self, robot_name):
        while True:
            keyboard_input = getch.getch()
            if keyboard_input == "q":
                break
            if keyboard_input in self.move_key_list:
                self.keyBoardMove(robot_name, self.move_dist, keyboard_input)
                continue
            if keyboard_input in self.rotate_key_list:
                self.keyBoardRotate(robot_name, self.rotate_angle, keyboard_input)
                continue

            print("RobotKeyboardController::keyBoardControl :")
            print("keyboard_input out of range!")
        return True

if __name__ == "__main__":
    robot_keyboard_controller = RobotKeyboardController()
    robot_keyboard_controller.keyBoardControl("kinect_camera_0")

