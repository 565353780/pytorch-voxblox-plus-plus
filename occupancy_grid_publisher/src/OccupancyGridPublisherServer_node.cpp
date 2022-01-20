#include "occupancy_grid_publisher/OccupancyGridPublisherServer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OccupancyGridPublisherServer");

  OccupancyGridPublisherServer occupancy_grid_publisher_server;

  ros::spin();

  return 1;
}

