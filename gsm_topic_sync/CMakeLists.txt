cmake_minimum_required(VERSION 3.0.2)
project(gsm_topic_sync)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_filters
  tf
  cv_bridge
)

catkin_package()

add_executable(${PROJECT_NAME}_node
  src/ImageConvert.cpp
  src/GSMTopicSync.cpp
  src/GSMTopicSync_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE ${OpenCV_LIBS} ${catkin_LIBRARIES})

target_include_directories(${PROJECT_NAME}_node PRIVATE include ${catkin_INCLUDE_DIRS})

target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

