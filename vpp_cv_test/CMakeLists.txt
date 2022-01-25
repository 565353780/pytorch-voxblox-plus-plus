cmake_minimum_required(VERSION 3.0.2)
project(
  vpp_cv_test
  LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vpp_cv_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

