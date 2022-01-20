#ifndef OCCUPANCY_GRID_PUBLISHER_SERVER_H
#define OCCUPANCY_GRID_PUBLISHER_SERVER_H

#include <ctime>

#include "occupancy_grid_publisher/OccupancyGridPublisher.h"
#include "tensorboard_logger_ros/ScalarToBool.h"

class OccupancyGridPublisherServer
{
public:
  OccupancyGridPublisherServer() :
    pointcloud_diff_sub_(nh_.subscribe<sensor_msgs::PointCloud2>(
          "depth_segmentation_node/object_segment", queue_size_,
          [this](const auto& msg){ this->addPointCloud2DiffCallback(msg); })),
    // occupancy_grid_publisher_(nh_.advertise<nav_msgs::OccupancyGrid>("gsm_node/occupancy_grid", queue_size_))
    occupancy_grid_pub_(nh_.advertise<nav_msgs::OccupancyGrid>("map", queue_size_)),
    tensorboard_logger_client_(nh_.serviceClient<tensorboard_logger_ros::ScalarToBool>("tensorboard_logger/log_scalar"))
  {
    start_clock_ = clock();
    log_idx_ = 0;
  }

private:
  bool addPointCloud2DiffCallback(const sensor_msgs::PointCloud2ConstPtr pointcloud2_diff);

  bool logTensorBoard(
      const std::string& name,
      const size_t& step,
      const float& value);

private:
  OccupancyGridPublisher occupancy_grid_publisher_;

  ros::NodeHandle nh_;
  std::uint32_t queue_size_ = 1;

  ros::Subscriber pointcloud_diff_sub_;
  ros::Publisher occupancy_grid_pub_;
  tf::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_pub_;

  ros::Time last_pub_tf_time_;

  clock_t start_clock_;
  size_t log_idx_;

  ros::ServiceClient tensorboard_logger_client_;
};

#endif // OCCUPANCY_GRID_PUBLISHER_SERVER_H
