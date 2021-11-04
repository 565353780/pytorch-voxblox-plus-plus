#ifndef ROBOT_POSITION_LOADER_H
#define ROBOT_POSITION_LOADER_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

class BBox3D
{
public:
  BBox3D()
  {
  }

  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};

class RobotPositionLoader
{
public:
  RobotPositionLoader()
  {
  }

  bool reset();

  bool updateRobotBBoxVec(
      const std::string &world_name,
      const std::vector<std::string> &robot_name_vec);

  bool getRobotBBoxVec(
      const std::string &world_name,
      const std::vector<std::string> &robot_name_vec,
      std::vector<BBox3D> &robot_bbox_vec);

  bool isPointInRobotBBox(
      const float &x,
      const float &y,
      const float &z);

private:
  bool isPointInBBox(
      const float &x,
      const float &y,
      const float &z,
      const BBox3D &bbox);

  tf::TransformListener tf_listener_;

  std::vector<BBox3D> robot_bbox_vec_;
};

#endif //ROBOT_POSITION_LOADER_H
