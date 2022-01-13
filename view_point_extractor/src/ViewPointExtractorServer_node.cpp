#include "ViewPointExtractorServer.h"
#include <cstdlib>
#include <ros/init.h>

int main(int argc, char** argv)
{
  std::cout << "Success run ViewPointExtractorServer!" << std::endl;

  ros::init(argc, argv, "ViewPointExtractorServer");

  ViewPointExtractorServer view_point_extractor_server;

  ros::spin();

  return 1;
}

