#ifndef ROBOT_POSITION_LOADER_H
#define ROBOT_POSITION_LOADER_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <gazebo_msgs/GetModelState.h>

#include "robot_position_loader/BBox3D.h"

class RobotPositionLoader
{
public:
  RobotPositionLoader() :
    get_model_state_client_(nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"))
  {
    history_save_num_ = 100;
  }

  bool reset();

  bool setWorldAndRobotParam(
      const std::string &world_name,
      const size_t &robot_num,
      const std::string &robot_name);


  bool isPointInRobotBBox(
      const float &x,
      const float &y,
      const float &z);

  bool updateRobotPose();
  bool updateRobotPoseWithTimeStamp(
      const ros::Time& stamp);

  const std::vector<robot_position_loader::BBox3D>& getRobotBBoxVec()
  {
    return robot_bbox_vec_;
  }

private:
  bool updateRobotPose(
      const size_t& robot_idx);
  bool updateRobotBBoxVec();

  bool isPointInBBox(
      const float &x,
      const float &y,
      const float &z,
      const robot_position_loader::BBox3D &bbox);

private:
  ros::NodeHandle nh_;

  ros::ServiceClient get_model_state_client_;

  tf::TransformListener tf_listener_;

  std::string world_name_;
  size_t robot_num_;
  std::string robot_name_;

  size_t history_save_num_;

  std::vector<std::deque<geometry_msgs::Pose>> robot_pose_vec_;
  std::vector<robot_position_loader::BBox3D> robot_bbox_vec_;
};

#endif //ROBOT_POSITION_LOADER_H
