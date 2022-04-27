#ifndef POINT_STATE_MANAGER_SERVER_H
#define POINT_STATE_MANAGER_SERVER_H

#include <ctime>

#include "PointStateManager.h"
#include "point_state_manager/PointToState.h"
#include "tensorboard_logger_ros/ScalarToBool.h"

class PointStateManagerServer
{
public:
  PointStateManagerServer() :
    add_new_point_server_(nh_.advertiseService("point_state_manager/add_new_point",
          &PointStateManagerServer::addNewPointCallback, this)),
    add_finish_point_server_(nh_.advertiseService("point_state_manager/add_finish_point",
          &PointStateManagerServer::addFinishPointCallback, this)),
    get_point_state_server_(nh_.advertiseService("point_state_manager/get_point_state",
          &PointStateManagerServer::getPointStateCallback, this)),
    tensorboard_logger_client_(nh_.serviceClient<tensorboard_logger_ros::ScalarToBool>("tensorboard_logger/log_scalar"))
  {
    start_clock_ = clock();
  }

private:
  bool addNewPointCallback(
      point_state_manager::PointToState::Request& req,
      point_state_manager::PointToState::Response& res);

  bool addFinishPointCallback(
      point_state_manager::PointToState::Request& req,
      point_state_manager::PointToState::Response& res);

  bool getPointStateCallback(
      point_state_manager::PointToState::Request& req,
      point_state_manager::PointToState::Response& res);

  bool logTensorBoard(
      const std::string& name,
      const size_t& step,
      const float& value);

private:
  PointStateManager point_state_manager_;

  ros::NodeHandle nh_;

  clock_t start_clock_;

  ros::ServiceServer add_new_point_server_;
  ros::ServiceServer add_finish_point_server_;
  ros::ServiceServer get_point_state_server_;
  ros::ServiceClient tensorboard_logger_client_;
};

#endif // POINT_STATE_MANAGER_SERVER_H

