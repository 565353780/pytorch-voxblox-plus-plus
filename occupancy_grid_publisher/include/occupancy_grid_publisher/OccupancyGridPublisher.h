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
  OccupancyGridPublisher() :
    pointcloud_diff_sub_(nh_.subscribe<sensor_msgs::PointCloud2>(
          "depth_segmentation_node/object_segment", queue_size_,
          [this](const auto& msg){ this->addPointCloud2DiffCallback(msg); })),
    // occupancy_grid_publisher_(nh_.advertise<nav_msgs::OccupancyGrid>("gsm_node/occupancy_grid", queue_size_))
    occupancy_grid_publisher_(nh_.advertise<nav_msgs::OccupancyGrid>("map", queue_size_))
  {
    initOccupancyGrid();
  }

private:
  bool initOccupancyGrid();

  bool addPointCloud2DiffCallback(const sensor_msgs::PointCloud2ConstPtr point_cloud2_diff);

  bool addPointCloud2Diff(
      const sensor_msgs::PointCloud2& pointcloud2_diff);

  ros::NodeHandle nh_;
  std::uint32_t queue_size_ = 1;

  ros::Subscriber pointcloud_diff_sub_;
  ros::Publisher occupancy_grid_publisher_;
  tf::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_pub_;

  float robot_height_min_ = 0.1;
  float robot_height_max_ = std::numeric_limits<float>::max();
  unsigned unknown_padding_size_ = 20;

  float current_x_min_;
  float current_x_max_;
  float current_y_min_;
  float current_y_max_;

  nav_msgs::OccupancyGrid last_occupancy_grid_;
  nav_msgs::OccupancyGrid occupancy_grid_;

  ros::Time last_pub_tf_time_;
};

#endif // OCCUPANCY_GRID_PUBLISHER_H
