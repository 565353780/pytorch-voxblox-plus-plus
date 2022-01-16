cmake_minimum_required(VERSION 3.0.2)

project(
  robot_keyboard_controller
  LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/RobotKeyboardController.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

