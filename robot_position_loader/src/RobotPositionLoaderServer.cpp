#include "robot_position_loader/RobotPositionLoaderServer.h"

bool RobotPositionLoaderServer::setWorldAndRobotParam(
    const std::string &world_name,
    const size_t &robot_num,
    const std::string &robot_name)
{
  if(!robot_position_loader_.setWorldAndRobotParam(world_name, robot_num, robot_name))
  {
    std::cout << "RobotPositionLoaderServer::setWorldAndRobotParam : " << std::endl <<
      "Input :\n" <<
      "\tworld_name = " << world_name << std::endl <<
      "\trobot_num = " << robot_num << std::endl <<
      "\trobot_name = " << robot_name << std::endl <<
      "setWorldAndRobotParam failed!" << std::endl;

    return false;
  }

  return true;
}

bool RobotPositionLoaderServer::getRobotBBoxVecCallback(
    robot_position_loader::GetRobotBBox3DVec::Request &req,
    robot_position_loader::GetRobotBBox3DVec::Response &res)
{
  std::cout << "Start RobotPositionLoader serve!" << std::endl;

  std::vector<robot_position_loader::BBox3D> robot_bbox_vec;
  robot_position_loader_.getRobotBBoxVec(robot_bbox_vec);

  res.robot_bbox_vec = robot_bbox_vec;

  // robot_bbox_vec_pub_.publish(robot_bbox_vec);

  return true;
}


