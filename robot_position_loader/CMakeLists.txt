cmake_minimum_required(VERSION 3.0.2)

project(
  robot_position_loader
  LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  genmsg
)

add_service_files(
  FILES
  GetRobotBBox3DVec.srv
)

add_message_files(
  FILES
  BBox3D.msg
  BBox3DVec.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/RobotPositionLoader.cpp
  src/RobotPositionLoaderServer.cpp
  src/RobotPositionLoaderServer_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_compile_features(${PROJECT_NAME}_node
  PRIVATE
    cxx_std_17
)

add_executable(try_${PROJECT_NAME}
  src/try_RobotPositionLoaderServer.cpp
)

target_link_libraries(try_${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(try_${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

