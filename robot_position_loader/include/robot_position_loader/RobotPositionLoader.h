#ifndef ROBOT_POSITION_LOADER_H
#define ROBOT_POSITION_LOADER_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "robot_position_loader/BBox3D.h"

class RobotPositionLoader
{
public:
  RobotPositionLoader()
  {
  }

  bool reset();

  bool setWorldAndRobotParam(
      const std::string &world_name,
      const size_t &robot_num,
      const std::string &robot_name);

  bool updateRobotBBoxVec();

  bool getRobotBBoxVec(
      std::vector<robot_position_loader::BBox3D> &robot_bbox_vec);

  bool isPointInRobotBBox(
      const float &x,
      const float &y,
      const float &z);

private:
  bool isPointInBBox(
      const float &x,
      const float &y,
      const float &z,
      const robot_position_loader::BBox3D &bbox);

  std::string world_name_;
  size_t robot_num_;
  std::string robot_name_;

  std::vector<robot_position_loader::BBox3D> robot_bbox_vec_;
};

#endif //ROBOT_POSITION_LOADER_H
