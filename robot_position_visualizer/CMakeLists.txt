cmake_minimum_required(VERSION 3.0.2)

project(
  robot_position_visualizer
  LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/RobotPositionVisualizer.cpp
  src/RobotPositionVisualizer_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${catkin_LIBRARIES}
)

target_compile_features(${PROJECT_NAME}_node
  PRIVATE
    cxx_std_17
)

