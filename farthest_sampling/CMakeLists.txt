cmake_minimum_required(VERSION 3.8)

project(
  farthest_sampling
  VERSION 0.1
  DESCRIPTION "description"
  LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Same as the project(LANGUAGES CUDA) but does not create any of the extra variables like CMAKE_CUDA_ARCHITECTURES
# Since we will detect cuda architecture automatically in detect_cuda_architectures function
# Or, the user should explicitly specify for which cuda architecture they want to build, set(CMAKE_CUDA_ARCHITECTURES xxx)
enable_language(CUDA)

include(DetectCudaArchitectures)
detect_cuda_architectures(CUDA_ARCHS)
set(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCHS})

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

find_package(PCL REQUIRED COMPONENTS common io)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS PCL
)

add_library(farthest_sampling_cu OBJECT)

target_sources(farthest_sampling_cu PRIVATE src/farthest_sampling.cu)

# CMAKE_CUDA_ARCHITECTURES variable only play its role with CMake >= 3.18
# We need to add compile flags manually
if(CMAKE_VERSION VERSION_LESS 3.18)
  # extract numbers from CMAKE_CUDA_ARCHITECTURES (86-real -> 86)
  string(REGEX MATCH "^[0-9]+" CUDA_ARCHS ${CMAKE_CUDA_ARCHITECTURES})
  target_compile_options(
    farthest_sampling_cu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-gencode arch=compute_${CUDA_ARCHS},code=sm_${CUDA_ARCHS}>)
endif()

# Add debug symbol for device code
if(CMAKE_BUILD_TYPE MATCHES "Debug|RelWithDebInfo")
  target_compile_options(
    farthest_sampling_cu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-G>)
endif()

set_target_properties(farthest_sampling_cu PROPERTIES POSITION_INDEPENDENT_CODE
                                                      ON)

add_library(${PROJECT_NAME} src/farthest_sampling.cpp)

target_link_libraries(
  ${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES}
                         farthest_sampling_cu)

target_include_directories(
  ${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
                         ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  catkin_add_gtest(test_farthest_sampling test/test_farthest_sampling.cpp)
  target_link_libraries(test_farthest_sampling ${PROJECT_NAME}
                        ${PCL_IO_LIBRARIES} Boost::filesystem)

  target_compile_definitions(test_farthest_sampling
    PRIVATE
      INPUT_POINT_CLOUD_PATH=${CMAKE_CURRENT_LIST_DIR}/test/ply/fragment.ply
      INPUT_POINT_CLOUD_DIR=${CMAKE_CURRENT_LIST_DIR}/test/ply/
      OUTPUT_POINT_CLOUD_PATH=${CMAKE_CURRENT_LIST_DIR}/test/output/sampled.ply
      OUTPUT_POINT_CLOUD_DIR=${CMAKE_CURRENT_LIST_DIR}/test/output/
  )
endif()
