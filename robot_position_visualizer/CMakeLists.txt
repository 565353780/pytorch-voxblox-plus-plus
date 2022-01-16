cmake_minimum_required(VERSION 3.0.2)

project(
  robot_position_visualizer
  LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/RobotPositionVisualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

