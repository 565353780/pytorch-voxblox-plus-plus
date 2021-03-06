#ifndef POINT_STATE_MANAGER_SERVER_H
#define POINT_STATE_MANAGER_SERVER_H

#include <ctime>

#include "PointStateManager.h"
#include "point_state_manager/PointVecToStateVec.h"
#include "tensorboard_logger_ros/ScalarToBool.h"

class PointStateManagerServer
{
public:
  PointStateManagerServer() :
    occupancy_grid_pub_(nh_.advertise<nav_msgs::OccupancyGrid>("task_map", queue_size_)),
    set_point_vec_state_server_(nh_.advertiseService("point_state_manager/set_point_vec_state",
          &PointStateManagerServer::setPointVecStateCallback, this)),
    get_point_state_vec_server_(nh_.advertiseService("point_state_manager/get_point_state_vec",
          &PointStateManagerServer::getPointStateVecCallback, this)),
    tensorboard_logger_client_(nh_.serviceClient<tensorboard_logger_ros::ScalarToBool>("tensorboard_logger/log_scalar"))
  {
    start_clock_ = clock();
  }

private:
  bool setPointVecStateCallback(
      point_state_manager::PointVecToStateVec::Request& req,
      point_state_manager::PointVecToStateVec::Response& res);

  bool getPointStateVecCallback(
      point_state_manager::PointVecToStateVec::Request& req,
      point_state_manager::PointVecToStateVec::Response& res);

  bool publishOccupancyMap();

  bool logTensorBoard(
      const std::string& name,
      const size_t& step,
      const float& value);

private:
  PointStateManager point_state_manager_;

  ros::NodeHandle nh_;
  std::uint32_t queue_size_ = 1;

  ros::Publisher occupancy_grid_pub_;
  tf2_ros::TransformBroadcaster tf_pub_;

  ros::Time last_pub_tf_time_;

  clock_t start_clock_;

  ros::ServiceServer set_point_vec_state_server_;
  ros::ServiceServer get_point_state_vec_server_;
  ros::ServiceClient tensorboard_logger_client_;
};

#endif // POINT_STATE_MANAGER_SERVER_H

