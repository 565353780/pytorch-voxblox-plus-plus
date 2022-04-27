#include "point_state_manager/PointStateManagerServer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointStateManagerServer");

  PointStateManagerServer point_state_manager_server;

  ros::spin();

  return 1;
}

