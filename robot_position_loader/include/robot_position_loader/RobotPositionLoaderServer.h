#ifndef ROBOT_POSITION_LOADER_SERVER_H
#define ROBOT_POSITION_LOADER_SERVER_H

#include "robot_position_loader/RobotPositionLoader.h"
#include "robot_position_loader/BBox3DVec.h"
#include "robot_position_loader/GetRobotBBox3DVec.h"

class RobotPositionLoaderServer
{
public:
  RobotPositionLoaderServer() :
    robot_position_loader_server_(nh_.advertiseService("robot_position_loader/get_robot_bbox_vec",
                                 &RobotPositionLoaderServer::getRobotBBoxVecCallback, this)),
    robot_bbox_vec_pub_(nh_.advertise<robot_position_loader::BBox3DVec>("robot_position_loader/robot_bbox_vec", queue_size_))
  {
  }

  bool setWorldAndRobotParam(
      const std::string &world_name,
      const size_t &robot_num,
      const std::string &robot_name);

private:
  bool updateRobotPose();

  bool getRobotBBoxVecCallback(
      robot_position_loader::GetRobotBBox3DVec::Request &req,
      robot_position_loader::GetRobotBBox3DVec::Response &res);

private:
  ros::NodeHandle nh_;
  uint32_t queue_size_ = 1;

  ros::ServiceServer robot_position_loader_server_;
  ros::Publisher robot_bbox_vec_pub_;

  RobotPositionLoader robot_position_loader_;
};

#endif //ROBOT_POSITION_LOADER_SERVER_H
