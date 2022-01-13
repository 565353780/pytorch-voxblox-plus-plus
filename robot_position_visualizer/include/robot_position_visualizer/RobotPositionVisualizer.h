#ifndef ROBOT_POSITION_VISUALIZER_H
#define ROBOT_POSITION_VISUALIZER_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetModelState.h>

class RobotPositionVisualizer
{
public:
  RobotPositionVisualizer() :
    get_model_state_client_(nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"))
  {
    history_save_num_ = 100;
  }

  bool reset();

  bool setWorldAndRobotParam(
      const std::string &world_name,
      const size_t &robot_num,
      const std::string &robot_name);

  bool updateRobotPose();

private:
  bool updateRobotPose(
      const size_t& robot_idx);

private:
  ros::NodeHandle nh_;

  ros::ServiceClient get_model_state_client_;

  std::string world_name_;
  size_t robot_num_;
  std::string robot_name_;

  size_t history_save_num_;

  std::vector<std::deque<geometry_msgs::Pose>> robot_pose_vec_;
};

#endif //ROBOT_POSITION_VISUALIZER_H
