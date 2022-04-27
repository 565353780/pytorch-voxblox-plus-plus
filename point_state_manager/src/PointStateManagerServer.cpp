#include "point_state_manager/PointStateManagerServer.h"

bool PointStateManagerServer::addNewPointCallback(
    point_state_manager::PointToState::Request& req,
    point_state_manager::PointToState::Response& res)
{
  const geometry_msgs::Point& new_point = req.point;
  if(!point_state_manager_.addNewPoint(new_point))
  {
    std::cout << "[ERROR][PointStateManagerServer::addNewPointCallback]\n" <<
      "\t addNewPoint failed!\n";

    return false;
  }

  return true;
}

bool PointStateManagerServer::addFinishPointCallback(
    point_state_manager::PointToState::Request& req,
    point_state_manager::PointToState::Response& res)
{
  const geometry_msgs::Point& finish_point = req.point;
  if(!point_state_manager_.addFinishPoint(finish_point))
  {
    std::cout << "[ERROR][PointStateManagerServer::addNewPointCallback]\n" <<
      "\t addNewPoint failed!\n";

    return false;
  }

  return true;
}

bool PointStateManagerServer::getPointStateCallback(
    point_state_manager::PointToState::Request& req,
    point_state_manager::PointToState::Response& res)
{
  const geometry_msgs::Point& query_point = req.point;
  const int point_state = point_state_manager_.getPointState(query_point);
  if(point_state == 0)
  {
    std::cout << "[ERROR][PointStateManagerServer::addNewPointCallback]\n" <<
      "\t addNewPoint failed!\n";

    return false;
  }

  res.state = point_state;

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

