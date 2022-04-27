#include "point_state_manager/PointStateManager.h"

bool PointStateManager::initOccupancyGrid()
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

bool PointStateManager::addNewPoint(
    const geometry_msgs::Point& point)
{
  current_x_min_ = std::fmin(current_x_min_, point.x - different_point_dist_min_);
  current_x_max_ = std::fmax(current_x_max_, point.x + different_point_dist_min_);
  current_y_min_ = std::fmin(current_y_min_, point.y - different_point_dist_min_);
  current_y_max_ = std::fmax(current_y_max_, point.y + different_point_dist_min_);

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

      occupancy_grid_.data[new_idx] = last_occupancy_grid_.data[col_start + col];
    }
  }

  const size_t point_half_length =
    std::floor(different_point_dist_min_ / occupancy_grid_.info.resolution);

  const int col =
    std::floor((point.x - float(new_origin_x)) / occupancy_grid_.info.resolution);
  const int row =
    std::floor((point.y - float(new_origin_y)) / occupancy_grid_.info.resolution);

  for(int i = -point_half_length; i <= point_half_length; ++i)
  {
    const int current_col = col + i;
    for(int j = -point_half_length; j <= point_half_length; ++j)
    {
      const int current_row = row + j;

      const int idx = current_row * occupancy_grid_.info.width + current_col;

      occupancy_grid_.data[idx] = 100;
    }
  }

  occupancy_grid_.header.frame_id = "map";

  last_occupancy_grid_ = occupancy_grid_;

  return true;
}

bool PointStateManager::addFinishPoint(
    const geometry_msgs::Point& point)
{
  return true;
}

bool PointStateManager::getPointState(
    const geometry_msgs::Point& point)
{
  return true;
}

