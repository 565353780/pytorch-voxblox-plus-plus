#ifndef POINT_STATE_MANAGER_H
#define POINT_STATE_MANAGER_H

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

class PointStateManager
{
public:
  PointStateManager()
  {
    initOccupancyGrid();
  }

  bool addNewPoint(
      const geometry_msgs::Point& point);

  bool addFinishPoint(
      const geometry_msgs::Point& point);

  bool getPointState(
      const geometry_msgs::Point& point);

  const nav_msgs::OccupancyGrid& getOccupancyGrid()
  {
    return occupancy_grid_;
  }

private:
  bool initOccupancyGrid();

  unsigned unknown_padding_size_ = 20;
  float pixel_area_ = 0.05 * 0.05;
  float different_point_dist_min_ = 0.2;

  float current_x_min_;
  float current_x_max_;
  float current_y_min_;
  float current_y_max_;

  nav_msgs::OccupancyGrid last_occupancy_grid_;
  nav_msgs::OccupancyGrid occupancy_grid_;
};

#endif // POINT_STATE_MANAGER_H

