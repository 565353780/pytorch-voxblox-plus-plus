#include "PointCloud2ToObjectVecConverterServer.h"

#include <ros/init.h>

int main(int argc, char** argv)
{
  std::cout << "Success run PointCloud2ToObjectVecConverterServer!" << std::endl;

  ros::init(argc, argv, "PointCloud2ToObjectVecConverterServer");

  PointCloud2ToObjectVecConverterServer pointcloud2_to_object_vec_converter_server;

  ros::spin();

  return 1;
}

