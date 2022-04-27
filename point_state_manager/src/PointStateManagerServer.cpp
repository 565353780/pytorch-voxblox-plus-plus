#include "point_state_manager/PointStateManagerServer.h"

bool PointStateManagerServer::addNewPointVecCallback(
    point_state_manager::PointVecToStateVec::Request& req,
    point_state_manager::PointVecToStateVec::Response& res)
{
  const std::vector<geometry_msgs::Point>& new_point_vec = req.point_vec;
  if(!point_state_manager_.addNewPointVec(new_point_vec))
  {
    std::cout << "[ERROR][PointStateManagerServer::addNewPointVecCallback]\n" <<
      "\t addNewPointVec failed!\n";

    return false;
  }

  occupancy_grid_pub_.publish(point_state_manager_.getOccupancyGrid());

  return true;
}

bool PointStateManagerServer::addFinishPointVecCallback(
    point_state_manager::PointVecToStateVec::Request& req,
    point_state_manager::PointVecToStateVec::Response& res)
{
  const std::vector<geometry_msgs::Point>& finish_point_vec = req.point_vec;
  if(!point_state_manager_.addFinishPointVec(finish_point_vec))
  {
    std::cout << "[ERROR][PointStateManagerServer::addFinishPointVecCallback]\n" <<
      "\t addFinishPointVec failed!\n";

    return false;
  }

  occupancy_grid_pub_.publish(point_state_manager_.getOccupancyGrid());

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

