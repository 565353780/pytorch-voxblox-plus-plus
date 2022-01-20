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

  current_x_min_ = std::numeric_limits<float>::max();
  current_x_max_ = std::numeric_limits<float>::min();
  current_y_min_ = std::numeric_limits<float>::max();
  current_y_max_ = std::numeric_limits<float>::min();

  return true;
}

bool OccupancyGridPublisher::addPointCloud2DiffCallback(
    const sensor_msgs::PointCloud2ConstPtr point_cloud2_diff)
{
  sensor_msgs::PointCloud point_cloud_diff;
  sensor_msgs::convertPointCloud2ToPointCloud(*point_cloud2_diff, point_cloud_diff);

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

  std::vector<tf::Vector3> trans_diff_point_vec;

  for(geometry_msgs::Point32& diff_point : point_cloud_diff.points)
  {
    const tf::Vector3 trans_diff_point = camera_to_map_transform(tf::Vector3(
          diff_point.x, diff_point.y, diff_point.z));
    trans_diff_point_vec.emplace_back(trans_diff_point);
  }

  // get the pointcloud's region
  auto [x_min_point, x_max_point] =
      std::minmax_element(trans_diff_point_vec.cbegin(), trans_diff_point_vec.cend(),
                          [](const auto& point1, const auto& point2) { return point1.x() < point2.x(); });
  auto [y_min_point, y_max_point] =
      std::minmax_element(trans_diff_point_vec.cbegin(), trans_diff_point_vec.cend(),
                          [](const auto& point1, const auto& point2) { return point1.y() < point2.y(); });
  auto [z_min_point, z_max_point] =
      std::minmax_element(trans_diff_point_vec.cbegin(), trans_diff_point_vec.cend(),
                          [](const auto& point1, const auto& point2) { return point1.z() < point2.z(); });

  current_x_min_ = std::fmin(current_x_min_, x_min_point->x());
  current_x_max_ = std::fmax(current_x_max_, x_max_point->x());
  current_y_min_ = std::fmin(current_y_min_, y_min_point->y());
  current_y_max_ = std::fmax(current_y_max_, y_max_point->y());

  for (const tf::Vector3& point : trans_diff_point_vec)
  {
    Point2D new_point;
    new_point.x = point.x();
    new_point.y = point.y();
    if (point.z() >= robot_height_min_ && point.z() <= robot_height_max_)
    {
      if(getPointDist2ToPointVec(new_point, obstacle_point2d_vec_) < point_dist2_min_)
      {
        continue;
      }
      obstacle_point2d_vec_.emplace_back(new_point);
    }
    else
    {
      if(getPointDist2ToPointVec(new_point, free_point2d_vec_) < point_dist2_min_)
      {
        continue;
      }
      free_point2d_vec_.emplace_back(new_point);
    }
  }

  // stretch out to contain some unknown pixels
  std::uint32_t new_width = std::ceil((current_x_max_ - current_x_min_) / occupancy_grid_.info.resolution);
  new_width += unknown_padding_size_;
  std::uint32_t new_height = std::ceil((current_y_max_ - current_y_min_) / occupancy_grid_.info.resolution);
  new_height += unknown_padding_size_;

  // ROS_INFO("Resizing occupancy_grid_, new size is : [%d, %d]", new_width, new_height);
  occupancy_grid_.info.width = std::max(new_width, occupancy_grid_.info.width);
  occupancy_grid_.info.height = std::max(new_height, occupancy_grid_.info.height);

  float new_origin_x = current_x_min_ - unknown_padding_size_ * occupancy_grid_.info.resolution / 2;
  new_origin_x = std::fmin(new_origin_x, occupancy_grid_.info.origin.position.x);

  float new_origin_y = current_y_min_ - unknown_padding_size_ * occupancy_grid_.info.resolution / 2;
  new_origin_y = std::fmin(new_origin_y, occupancy_grid_.info.origin.position.y);

  occupancy_grid_.info.origin.position.x = new_origin_x;
  occupancy_grid_.info.origin.position.y = new_origin_y;

  occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);

  std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);
  // if have obstacle, the value is 100, otherwise is 0

  for (const Point2D& point : free_point2d_vec_)
  {
    std::size_t col = std::floor((point.x - new_origin_x) / occupancy_grid_.info.resolution);
    std::size_t row = std::floor((point.y - new_origin_y) / occupancy_grid_.info.resolution);
    occupancy_grid_.data[row * occupancy_grid_.info.width + col] = 0;
  }
  for (const Point2D& point : obstacle_point2d_vec_)
  {
    std::size_t col = std::floor((point.x - new_origin_x) / occupancy_grid_.info.resolution);
    std::size_t row = std::floor((point.y - new_origin_y) / occupancy_grid_.info.resolution);
    occupancy_grid_.data[row * occupancy_grid_.info.width + col] = 100;
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

float OccupancyGridPublisher::getPointDist2ToPointVec(
    const Point2D& point,
    const std::vector<Point2D>& point_vec)
{
  float min_dist2_to_point_vec = std::numeric_limits<float>::max();

  if(point_vec.size() == 0)
  {
    return min_dist2_to_point_vec;
  }

  for(const Point2D& exist_point : point_vec)
  {
    const float current_point_x_diff = point.x - exist_point.x;
    const float current_point_y_diff = point.y - exist_point.y;

    const float current_dist =
      current_point_x_diff * current_point_x_diff +
      current_point_y_diff * current_point_y_diff;

    min_dist2_to_point_vec = std::min(min_dist2_to_point_vec, current_dist);
  }

  return min_dist2_to_point_vec;
}

