cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_diff)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vpp_msgs
  tensorboard_logger_ros
)

catkin_install_python(PROGRAMS
  scripts/PointCloudDiff.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

