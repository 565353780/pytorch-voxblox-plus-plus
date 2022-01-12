cmake_minimum_required(VERSION 3.0)

project(glog_catkin VERSION 0.5.0 LANGUAGES CXX)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

set(VERSION 0.5.0)
ExternalProject_Add(glog_src
  URL https://github.com/google/glog/archive/v${VERSION}.zip
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=Release
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/glog
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libglog*")
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES glog)
