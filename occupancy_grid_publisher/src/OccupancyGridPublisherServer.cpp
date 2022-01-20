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

  return true;
}

