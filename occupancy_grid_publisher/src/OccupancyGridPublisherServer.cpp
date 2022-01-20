#include "occupancy_grid_publisher/OccupancyGridPublisherServer.h"

bool OccupancyGridPublisherServer::addPointCloud2DiffCallback(
    const sensor_msgs::PointCloud2ConstPtr pointcloud2_diff)
{
  sensor_msgs::PointCloud pointcloud_diff;
  sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_diff, pointcloud_diff);

  ros::Time get_tf_time = pointcloud_diff.header.stamp;

  tf::StampedTransform camera_to_map_transform;
  try
  {
    tf_listener_.lookupTransform("map", pointcloud_diff.header.frame_id,
        get_tf_time, camera_to_map_transform);
  }
  catch(tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1).sleep();
    return true;
  }

  if(!occupancy_grid_publisher_.addPointCloudDiff(
        pointcloud_diff,
        camera_to_map_transform))
  {
    std::cout << "OccupancyGridPublisherServer::addPointCloud2DiffCallback :\n" <<
      "addPointCloudDiff failed!\n";

    return false;
  }

  const nav_msgs::OccupancyGrid& occupancy_grid =
    occupancy_grid_publisher_.getOccupancyGrid();

  // geometry_msgs::TransformStamped transform_map_to_occupancy_grid;
  // transform_map_to_occupancy_grid.header.frame_id = "map";
  // transform_map_to_occupancy_grid.child_frame_id = occupancy_grid.header.frame_id;
  // transform_map_to_occupancy_grid.transform.translation.x = 0;
  // transform_map_to_occupancy_grid.transform.translation.y = 0;
  // transform_map_to_occupancy_grid.transform.translation.z = 0;
  // transform_map_to_occupancy_grid.transform.rotation.x = 0;
  // transform_map_to_occupancy_grid.transform.rotation.y = 0;
  // transform_map_to_occupancy_grid.transform.rotation.z = 0;
  // transform_map_to_occupancy_grid.transform.rotation.w = 1;
  // transform_map_to_occupancy_grid.header.stamp = occupancy_grid.header.stamp;

  if(occupancy_grid.header.stamp == last_pub_tf_time_)
  {
    return true;
  }

  occupancy_grid_pub_.publish(occupancy_grid);
  // tf_pub_.sendTransform(transform_map_to_occupancy_grid);

  last_pub_tf_time_ = occupancy_grid.header.stamp;

  if((clock() - start_clock_) / CLOCKS_PER_SEC < log_idx_)
  {
    return true;
  }

  const size_t obstacle_pixel_num_ = occupancy_grid_publisher_.getObstaclePixelNum();
  const size_t free_pixel_num_ = occupancy_grid_publisher_.getFreePixelNum();

  if(!logTensorBoard(
        "OccupancyGridPublisherServer/obstacle_area",
        log_idx_,
        obstacle_pixel_num_))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback :\n" <<
      "logTensorBoard for obstacle_area failed!" << std::endl;

    return false;
  }

  if(!logTensorBoard(
        "OccupancyGridPublisherServer/free_area",
        log_idx_,
        free_pixel_num_))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback :\n" <<
      "logTensorBoard for free_area failed!" << std::endl;

    return false;
  }

  if(!logTensorBoard(
        "OccupancyGridPublisherServer/scene_area",
        log_idx_,
        obstacle_pixel_num_ + free_pixel_num_))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback :\n" <<
      "logTensorBoard for scene_area failed!" << std::endl;

    return false;
  }

  ++log_idx_;

  return true;
}

bool OccupancyGridPublisherServer::logTensorBoard(
    const std::string& name,
    const size_t& step,
    const float& value)
{
  tensorboard_logger_ros::ScalarToBool tensorboard_logger_serve;

  // RUN_LOG PARAM
  tensorboard_logger_serve.request.scalar.name = name;
  tensorboard_logger_serve.request.scalar.step = step;
  tensorboard_logger_serve.request.scalar.value = value;

  if (!tensorboard_logger_client_.call(tensorboard_logger_serve))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::logTensorBoard :\n" <<
      "call tensorboard_logger_server failed!\n";

    return false;
  }

  if(!tensorboard_logger_serve.response.success)
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::logTensorBoard :\n" <<
      "tensorboard_logger_server log failed!\n";

    return false;
  }

  return true;
}

