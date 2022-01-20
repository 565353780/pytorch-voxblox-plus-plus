cmake_minimum_required(VERSION 3.0.2)
project(occupancy_grid_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  genmsg
  vpp_msgs
  tf
  tf2_ros
  tensorboard_logger_ros
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  src/OccupancyGridPublisher.cpp
  src/OccupancyGridPublisherServer.cpp
  src/OccupancyGridPublisherServer_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${catkin_LIBRARIES}
)

target_compile_features(${PROJECT_NAME}_node
  PRIVATE
    cxx_std_17
)

