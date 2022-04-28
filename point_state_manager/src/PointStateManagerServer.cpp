#include "point_state_manager/PointStateManagerServer.h"

bool PointStateManagerServer::setPointVecStateCallback(
    point_state_manager::PointVecToStateVec::Request& req,
    point_state_manager::PointVecToStateVec::Response& res)
{
  if(!point_state_manager_.setPointVecState(
        req.point_vec, req.state, req.effect_radius))
  {
    std::cout << "[ERROR][PointStateManagerServer::addNewPointVecCallback]\n" <<
      "\t setPointVecState failed!\n";

    return false;
  }

  if(!publishOccupancyMap())
  {
    std::cout << "[ERROR][PointStateManagerServer::addNewPointVecCallback]\n" <<
      "\t publishOccupancyMap failed!\n";

    return false;
  }

  return true;
}

bool PointStateManagerServer::getPointStateVecCallback(
    point_state_manager::PointVecToStateVec::Request& req,
    point_state_manager::PointVecToStateVec::Response& res)
{
  const std::vector<geometry_msgs::Point>& query_point_vec = req.point_vec;
  std::vector<int> state_vec;
  if(!point_state_manager_.getPointStateVec(query_point_vec, state_vec))
  {
    std::cout << "[ERROR][PointStateManagerServer::getPointStateVecCallback]\n" <<
      "\t getPointStateVec failed!\n";

    return false;
  }

  res.state_vec = state_vec;

  return true;
}

bool PointStateManagerServer::publishOccupancyMap()
{
  const nav_msgs::OccupancyGrid& occupancy_grid =
    point_state_manager_.getOccupancyGrid();

  occupancy_grid_pub_.publish(occupancy_grid);

  if(last_pub_tf_time_ == occupancy_grid.header.stamp)
  {
    return true;
  }
  geometry_msgs::TransformStamped transform_map_to_occupancy_grid;
  transform_map_to_occupancy_grid.header.frame_id = "map";
  transform_map_to_occupancy_grid.child_frame_id = occupancy_grid.header.frame_id;
  transform_map_to_occupancy_grid.transform.translation.x = 0;
  transform_map_to_occupancy_grid.transform.translation.y = 0;
  transform_map_to_occupancy_grid.transform.translation.z = 0;
  transform_map_to_occupancy_grid.transform.rotation.x = 0;
  transform_map_to_occupancy_grid.transform.rotation.y = 0;
  transform_map_to_occupancy_grid.transform.rotation.z = 0;
  transform_map_to_occupancy_grid.transform.rotation.w = 1;
  transform_map_to_occupancy_grid.header.stamp = occupancy_grid.header.stamp;
  tf_pub_.sendTransform(transform_map_to_occupancy_grid);

  last_pub_tf_time_ = occupancy_grid.header.stamp;

  return true;
}

bool PointStateManagerServer::logTensorBoard(
    const std::string& name,
    const size_t& step,
    const float& value)
{
  tensorboard_logger_ros::ScalarToBool tensorboard_logger_serve;

  tensorboard_logger_serve.request.scalar.name = name;
  tensorboard_logger_serve.request.scalar.step = step;
  tensorboard_logger_serve.request.scalar.value = value;

  if (!tensorboard_logger_client_.call(tensorboard_logger_serve))
  {
    std::cout << "PointStateManagerServer::logTensorBoard :\n" <<
      "call tensorboard_logger_server failed!\n";

    return false;
  }

  if(!tensorboard_logger_serve.response.success)
  {
    std::cout << "PointStateManagerServer::logTensorBoard :\n" <<
      "tensorboard_logger_server log failed!\n";

    return false;
  }

  return true;
}

