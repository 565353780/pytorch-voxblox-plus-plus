cmake_minimum_required(VERSION 3.0.2)

project(
  view_point_extractor
  LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  genmsg
  message_generation
  vpp_msgs
  grnet_detect
  pointcloud2_to_object_vec_converter
  farthest_sampling
  tensorboard_logger_ros
)

find_package(PCL REQUIRED)

add_service_files(
  FILES
  PC2ToViewPointVec.srv
)

add_message_files(
  FILES
  ViewPoint.msg
  ViewPointVec.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/EasyPCLVisualizer.cpp
  src/ViewPointSaver.cpp
  src/ViewPointExtractor.cpp
  src/ViewPointExtractorServer.cpp
  src/ViewPointExtractorServer_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${PCL_LIBRARIES}
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
  src/try_ViewPointExtractorServer.cpp
)

target_link_libraries(try_${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(try_${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

