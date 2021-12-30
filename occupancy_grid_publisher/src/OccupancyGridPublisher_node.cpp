#include "occupancy_grid_publisher/OccupancyGridPublisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OccupancyGridPublisher");

  OccupancyGridPublisher occupancy_grid_publisher;

  ros::spin();

  return 1;
}

