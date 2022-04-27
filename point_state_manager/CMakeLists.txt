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
  tf
  tf2_ros
  tensorboard_logger_ros
)

add_service_files(
  FILES
  PointVecToStateVec.srv
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

add_executable(try_${PROJECT_NAME}
  src/try_PointStateManagerServer.cpp
)

target_link_libraries(try_${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(try_${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

