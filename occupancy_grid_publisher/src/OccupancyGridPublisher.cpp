#include "occupancy_grid_publisher/OccupancyGridPublisher.h"

bool OccupancyGridPublisher::initOccupancyGrid()
{
  last_occupancy_grid_.info.width = 1;
  last_occupancy_grid_.info.height = 1;
  last_occupancy_grid_.info.origin.position.x = 0;
  last_occupancy_grid_.info.origin.position.y = 0;
  last_occupancy_grid_.info.origin.position.z = 0;
  last_occupancy_grid_.info.origin.orientation.x = 0;
  last_occupancy_grid_.info.origin.orientation.y = 0;
  last_occupancy_grid_.info.origin.orientation.z = 0;
  last_occupancy_grid_.info.origin.orientation.w = 1;
  last_occupancy_grid_.info.resolution = 0.05;
  last_occupancy_grid_.data.resize(last_occupancy_grid_.info.width * last_occupancy_grid_.info.height);
  std::fill(last_occupancy_grid_.data.begin(), last_occupancy_grid_.data.end(), -1);

  occupancy_grid_.info.width = 1;
  occupancy_grid_.info.height = 1;
  occupancy_grid_.info.origin.position.x = 0;
  occupancy_grid_.info.origin.position.y = 0;
  occupancy_grid_.info.origin.position.z = 0;
  occupancy_grid_.info.origin.orientation.x = 0;
  occupancy_grid_.info.origin.orientation.y = 0;
  occupancy_grid_.info.origin.orientation.z = 0;
  occupancy_grid_.info.origin.orientation.w = 1;
  occupancy_grid_.info.resolution = 0.05;
  occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);
  std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);

  current_x_min_ = 0;
  current_x_max_ = 0;
  current_y_min_ = 0;
  current_y_max_ = 0;

  return true;
}

bool OccupancyGridPublisher::addPointCloud2DiffCallback(
    const sensor_msgs::PointCloud2ConstPtr point_cloud2_diff)
{
  return addPointCloud2Diff(*point_cloud2_diff);
}

bool OccupancyGridPublisher::addPointCloud2Diff(
    const sensor_msgs::PointCloud2& pointcloud2_diff)
{
  sensor_msgs::PointCloud point_cloud_diff;
  sensor_msgs::convertPointCloud2ToPointCloud(pointcloud2_diff, point_cloud_diff);

  occupancy_grid_.header = point_cloud_diff.header;
  occupancy_grid_.header.frame_id = "gsm_occupancy_grid";
  occupancy_grid_.info.map_load_time = point_cloud_diff.header.stamp;

  ros::Time get_tf_time = point_cloud_diff.header.stamp;

  tf::StampedTransform camera_to_map_transform;
  try
  {
    tf_listener_.lookupTransform("map", point_cloud_diff.header.frame_id,
        get_tf_time, camera_to_map_transform);
  }
  catch(tf::TransformException ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1).sleep();
    return true;
  }

  std::vector<geometry_msgs::Point32> trans_diff_point_vec;
  for(geometry_msgs::Point32& diff_point : point_cloud_diff.points)
  {
    const tf::Vector3 trans_diff_vec3 = camera_to_map_transform(tf::Vector3(
          diff_point.x, diff_point.y, diff_point.z));
    geometry_msgs::Point32 trans_diff_point;
    trans_diff_point.x = float(trans_diff_vec3.x());
    trans_diff_point.y = float(trans_diff_vec3.y());
    trans_diff_point.z = float(trans_diff_vec3.z());
    trans_diff_point_vec.emplace_back(trans_diff_point);
  }

  // get the pointcloud's region
  auto [x_min_point, x_max_point] =
      std::minmax_element(trans_diff_point_vec.cbegin(), trans_diff_point_vec.cend(),
                          [](const auto& point1, const auto& point2) { return point1.x < point2.x; });
  auto [y_min_point, y_max_point] =
      std::minmax_element(trans_diff_point_vec.cbegin(), trans_diff_point_vec.cend(),
                          [](const auto& point1, const auto& point2) { return point1.y < point2.y; });

  current_x_min_ = std::min(current_x_min_, x_min_point->x);
  current_x_max_ = std::max(current_x_max_, x_max_point->x);
  current_y_min_ = std::min(current_y_min_, y_min_point->y);
  current_y_max_ = std::max(current_y_max_, y_max_point->y);

  // stretch out to contain some unknown pixels
  std::uint32_t new_width =
    std::ceil((current_x_max_ - current_x_min_ + 1.0) / occupancy_grid_.info.resolution);
  new_width += unknown_padding_size_;
  std::uint32_t new_height =
    std::ceil((current_y_max_ - current_y_min_ + 1.0) / occupancy_grid_.info.resolution);
  new_height += unknown_padding_size_;

  // ROS_INFO("Resizing occupancy_grid_, new size is : [%d, %d]", new_width, new_height);
  occupancy_grid_.info.width = new_width;
  occupancy_grid_.info.height = new_height;

  const int last_origin_x = std::floor(last_occupancy_grid_.info.origin.position.x);
  const int last_origin_y = std::floor(last_occupancy_grid_.info.origin.position.y);

  const int new_origin_x =
    std::floor(current_x_min_ - unknown_padding_size_ * occupancy_grid_.info.resolution / 2);
  const int new_origin_y =
    std::floor(current_y_min_ - unknown_padding_size_ * occupancy_grid_.info.resolution / 2);

  occupancy_grid_.info.origin.position.x = double(new_origin_x);
  occupancy_grid_.info.origin.position.y = double(new_origin_y);

  occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height, -1);
  std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);

  const int origin_x_diff = (last_origin_x - new_origin_x) / occupancy_grid_.info.resolution;
  const int origin_y_diff = (last_origin_y - new_origin_y) / occupancy_grid_.info.resolution;

  for(size_t row = 0; row < last_occupancy_grid_.info.height; ++row)
  {
    const size_t col_start = row * last_occupancy_grid_.info.width;
    const size_t new_row = row + origin_y_diff;
    const size_t new_col_start = new_row * occupancy_grid_.info.width;

    if(new_col_start >= occupancy_grid_.data.size())
    {
      break;
    }

    for(size_t col = 0; col < last_occupancy_grid_.info.width; ++col)
    {
      const size_t new_col = col + origin_x_diff;
      const size_t new_idx = new_col_start + new_col;

      if(new_idx >= occupancy_grid_.data.size())
      {
        break;
      }

      occupancy_grid_.data[new_idx] =
        last_occupancy_grid_.data[col_start + col];
    }
  }

  for(const geometry_msgs::Point32& trans_diff_point : trans_diff_point_vec)
  {
    const int col =
      std::floor((trans_diff_point.x - float(new_origin_x)) / occupancy_grid_.info.resolution);
    const int row =
      std::floor((trans_diff_point.y - float(new_origin_y)) / occupancy_grid_.info.resolution);

    const int idx = row * occupancy_grid_.info.width + col;

    if(occupancy_grid_.data[idx] != -1)
    {
      continue;
    }

    // if have obstacle, the value is 100, otherwise is 0
    if(trans_diff_point.z >= robot_height_min_ && trans_diff_point.z <= robot_height_max_)
    {
      occupancy_grid_.data[idx] = 100;
    }
    else
    {
      occupancy_grid_.data[idx] = 1;
    }
  }

  occupancy_grid_.header.frame_id = "map";

  last_occupancy_grid_ = occupancy_grid_;

  // geometry_msgs::TransformStamped transform_map_to_occupancy_grid;
  // transform_map_to_occupancy_grid.header.frame_id = "map";
  // transform_map_to_occupancy_grid.child_frame_id = occupancy_grid_.header.frame_id;
  // transform_map_to_occupancy_grid.transform.translation.x = 0;
  // transform_map_to_occupancy_grid.transform.translation.y = 0;
  // transform_map_to_occupancy_grid.transform.translation.z = 0;
  // transform_map_to_occupancy_grid.transform.rotation.x = 0;
  // transform_map_to_occupancy_grid.transform.rotation.y = 0;
  // transform_map_to_occupancy_grid.transform.rotation.z = 0;
  // transform_map_to_occupancy_grid.transform.rotation.w = 1;
  // transform_map_to_occupancy_grid.header.stamp = occupancy_grid_.header.stamp;

  if(occupancy_grid_.header.stamp == last_pub_tf_time_)
  {
    return true;
  }

  occupancy_grid_publisher_.publish(occupancy_grid_);
  // tf_pub_.sendTransform(transform_map_to_occupancy_grid);

  last_pub_tf_time_ = occupancy_grid_.header.stamp;

  return true;
}

