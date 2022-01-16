cmake_minimum_required(VERSION 3.0.2)
project(pointcloud2_to_object_vec_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  genmsg
  vpp_msgs
  message_generation
  tensorboard_logger_ros
)

find_package(PCL REQUIRED)

add_service_files(
  FILES
  PC2ToOBJS.srv
)

add_message_files(
  FILES
  PointCloud2Vec.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/PointCloud2ToObjectVecConverter.cpp
  src/PointCloud2ToObjectVecConverterServer.cpp
  src/PointCloud2ToObjectVecConverterServer_node.cpp
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
  src/try_PointCloud2ToObjectVecConverterServer.cpp
)

target_link_libraries(try_${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(try_${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

