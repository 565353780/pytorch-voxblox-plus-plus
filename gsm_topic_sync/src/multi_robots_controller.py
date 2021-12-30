#!/usr/bin/env python
#coding=utf-8
     
import rospy
from gazebo_msgs.srv import *

from math import cos, pi, sin, sqrt

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

class MultiRobotController:
    def __init__(self, robot_base_name, robot_num):        
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.objstate = SetModelStateRequest()
        
        self.robot_base_name = robot_base_name
        self.robot_num = robot_num
        self.robot_name_list = []
        
        self.initRobotNameList()
        
        self.initRequestObject()
        
    def initRobotNameList(self):
        self.robot_name_list = []
        
        for robot_idx in range(self.robot_num):
            self.robot_name_list.append(self.robot_base_name + str(robot_idx))
    
    def initRequestObject(self):
        self.objstate.model_state.twist.linear.x = 0.0
        self.objstate.model_state.twist.linear.y = 0.0
        self.objstate.model_state.twist.linear.z = 0.0
        self.objstate.model_state.twist.angular.x = 0.0
        self.objstate.model_state.twist.angular.y = 0.0
        self.objstate.model_state.twist.angular.z = 0.0
        self.objstate.model_state.reference_frame = "map"
        
    def waitTopic(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        
    def compute_face_to_quaternion(self, face_to):
        """ Compute the Quaternion of the move we need
        Args:
            face_to: list/tuple [x, y, z]
        Return:
            [x, y, z, w]
        """

        xy = sqrt(face_to[0] * face_to[0] + face_to[1] * face_to[1])
        xyz = sqrt(face_to[0] * face_to[0] + face_to[1] * face_to[1] + face_to[2] * face_to[2])
        if xyz == 0:
            return [0, 0, 0, 1]
        if xy == 0:
            if face_to[2] > 0:
                return [0, -1 / sqrt(2), 0, 1 / sqrt(2)]
            return [0, 1 / sqrt(2), 0, 1 / sqrt(2)]
        else:
            cos_fai = xy / xyz

            cos_theta = face_to[0] / xy
            sin_theta = face_to[1] / xy

            cos_half_fai = sqrt((1 + cos_fai) / 2)
            if face_to[2] > 0:
                sin_half_fai = sqrt((1 - cos_fai) / 2)
            else:
                sin_half_fai = - sqrt((1 - cos_fai) / 2)

            cos_half_theta = sqrt((1 + cos_theta) / 2)
            if face_to[1] > 0:
                sin_half_theta = sqrt((1 - cos_theta) / 2)
            else:
                sin_half_theta = - sqrt((1 - cos_theta) / 2)

            return [sin_half_fai * (cos_half_theta * sin_theta - sin_half_theta * cos_theta),
                    sin_half_fai * (sin_half_theta * sin_theta - cos_half_theta * cos_theta),
                    cos_half_fai * sin_half_theta,
                    cos_half_fai * cos_half_theta]
        
    def computeCameraState(self, position, face_to, safe_dist):
        """ Compute the fetch base(_link) target pose of the move we need
        Args:
            position: list/tuple [x, y, z]
            face_to: list/tuple [x, y, z]
            safe_dist: float
        Return:
            position, face_to_quaterion
        """
        face_to_vector_len = sqrt(face_to[0] ** 2 + face_to[1] ** 2 + face_to[2] ** 2)
        if face_to_vector_len == 0:
            face_to_vector_len = 1

        unit_face_to_vector = [face_to[0] / face_to_vector_len, face_to[1] / face_to_vector_len, face_to[2] / face_to_vector_len]

        return [position[0] -  safe_dist * unit_face_to_vector[0],
                position[1] -  safe_dist * unit_face_to_vector[1],
                position[2] -  safe_dist * unit_face_to_vector[2]], \
                self.compute_face_to_quaternion(unit_face_to_vector)
    
    def setRobotModel(self, name, position, face_to, safe_dist):
        target_position, target_face_to_quaterion = self.computeCameraState(position, face_to, safe_dist)
        
        self.objstate.model_state.model_name = name
        self.objstate.model_state.pose.position.x = target_position[0]
        self.objstate.model_state.pose.position.y = target_position[1]
        self.objstate.model_state.pose.position.z = target_position[2]
        self.objstate.model_state.pose.orientation.x = target_face_to_quaterion[0]
        self.objstate.model_state.pose.orientation.y = target_face_to_quaterion[1]
        self.objstate.model_state.pose.orientation.z = target_face_to_quaterion[2]
        self.objstate.model_state.pose.orientation.w = target_face_to_quaterion[3]
     
        result = self.set_state_service(self.objstate)
        
        return result
    
    def setAllRobotModels(self, position, face_to, position_style):
        """
        Set all robot models by 2 different robot position styles.
        
        input:
        position([x, y, z]): mean position of all robots
        face_to([x, y, z]): face to of the first robot
        position_style(str): circle -> all robots are on a circle. line -> all robots are on a line
        
        output:
        result_list([result]): results of all robots after set models
        """
        
        circle_radius = 0.2
        line_dist = 0.3
        
        face_to_length = sqrt(face_to[0] ** 2 + face_to[1] ** 2 + face_to[2] ** 2)
        
        unit_face_to = [face_to[0] / face_to_length, face_to[1] / face_to_length, face_to[2] / face_to_length]
        
        result_list = []
        
        for robot_idx in range(self.robot_num):
            robot_name = self.robot_name_list[robot_idx]
            current_face_to = []
            current_position = []
            
            if position_style == "circle":
                current_angle = 2.0 * robot_idx * pi / self.robot_num
                current_face_to = [(unit_face_to[0] * cos(current_angle) - unit_face_to[1] * sin(current_angle)),
                                   (unit_face_to[0] * sin(current_angle) + unit_face_to[1] * cos(current_angle)),
                                   face_to[2]]
                current_position = [position[0] + circle_radius * current_face_to[0],
                                    position[1] + circle_radius * current_face_to[1],
                                    position[2]]
            elif position_style == "line":
                current_angle = pi / 2.0
                current_face_to = unit_face_to
                current_position = [position[0] + (self.robot_num / 2.0 - robot_idx) * line_dist * (-current_face_to[1]),
                                    position[1] + (self.robot_num / 2.0 - robot_idx) * line_dist * (current_face_to[0]),
                                    position[2]]
            
            current_result = self.setRobotModel(self.robot_name_list[robot_idx],
                                                current_position,
                                                current_face_to,
                                                0)
            result_list.append(current_result)
        
        return result_list
            
if __name__ == "__main__":
    rospy.init_node('multi_robots_controller')

    multi_robot_controller = MultiRobotController(robot_base_name="srobot",
                                                  robot_num=4)

    multi_robot_controller.waitTopic()

    # multi_robot_controller.setAllRobotModels(position=[2.5, 1.5, 0.5],
    #                                          face_to=[1, 0, 0],
    #                                          position_style="circle")
    
    current_position = [2.5, 1.5, 0.5]
    input_word = "start"
    
    getch = _Getch()
    
    print("input direction :")
    
    while not input_word == "q":
        if input_word == "start":
            multi_robot_controller.setAllRobotModels(position=current_position,
                                                     face_to=[1, 0, 0],
                                                     position_style="circle")
        elif input_word == "w":
            current_position[0] -= 1
            multi_robot_controller.setAllRobotModels(position=current_position,
                                                     face_to=[1, 0, 0],
                                                     position_style="circle")
        elif input_word == "a":
            current_position[1] -= 1
            multi_robot_controller.setAllRobotModels(position=current_position,
                                                     face_to=[1, 0, 0],
                                                     position_style="circle")
        elif input_word == "s":
            current_position[0] += 1
            multi_robot_controller.setAllRobotModels(position=current_position,
                                                     face_to=[1, 0, 0],
                                                     position_style="circle")
        elif input_word == "d":
            current_position[1] += 1
            multi_robot_controller.setAllRobotModels(position=current_position,
                                                     face_to=[1, 0, 0],
                                                     position_style="circle")
            
        input_word = getch()