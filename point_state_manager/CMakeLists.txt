cmake_minimum_required(VERSION 3.0.2)
project(point_state_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  genmsg
  nav_msgs
  vpp_msgs
  tensorboard_logger_ros
)

add_service_files(
  FILES
  PointToState.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  src/PointStateManager.cpp
  src/PointStateManagerServer.cpp
  src/PointStateManagerServer_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${catkin_LIBRARIES}
)

target_compile_features(${PROJECT_NAME}_node
  PRIVATE
    cxx_std_17
)

