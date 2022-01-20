#ifndef OCCUPANCY_GRID_PUBLISHER_H
#define OCCUPANCY_GRID_PUBLISHER_H

#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OccupancyGridPublisher
{
public:
  OccupancyGridPublisher()
  {
    initOccupancyGrid();
  }

  bool addPointCloudDiff(
      const sensor_msgs::PointCloud& pointcloud_diff,
      const tf::StampedTransform& camera_to_map_transform);

  const nav_msgs::OccupancyGrid& getOccupancyGrid()
  {
    return occupancy_grid_;
  }

private:
  bool initOccupancyGrid();

  float robot_height_min_ = 0.1;
  float robot_height_max_ = std::numeric_limits<float>::max();
  unsigned unknown_padding_size_ = 20;

  float current_x_min_;
  float current_x_max_;
  float current_y_min_;
  float current_y_max_;

  nav_msgs::OccupancyGrid last_occupancy_grid_;
  nav_msgs::OccupancyGrid occupancy_grid_;
};

#endif // OCCUPANCY_GRID_PUBLISHER_H
