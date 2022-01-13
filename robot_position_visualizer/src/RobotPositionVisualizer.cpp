#include "robot_position_visualizer/RobotPositionVisualizer.h"

bool RobotPositionVisualizer::reset()
{
  robot_pose_vec_.clear();

  return true;
}

bool RobotPositionVisualizer::setWorldAndRobotParam(
    const std::string &world_name,
    const size_t &robot_num,
    const std::string &robot_name)
{
  world_name_ = world_name;
  robot_num_ = robot_num;
  robot_name_ = robot_name;

  return true;
}

bool RobotPositionVisualizer::updateRobotPose()
{
  if(robot_num_ == 0)
  {
    std::cout << "RobotPositionVisualizer::updateRobotPose :\n" <<
      "robot_num_ is 0!\n";

    return false;
  }

  robot_pose_vec_.resize(robot_num_);

  for(size_t i = 0; i < robot_num_; ++i)
  {
    if(!updateRobotPose(i))
    {
      std::cout << "RobotPositionVisualizer::updateRobotPose :\n" <<
        "updateRobotPose for robot " << i << " failed!\n";

      return false;
    }
  }

  return true;
}

bool RobotPositionVisualizer::updateRobotPose(
    const size_t& robot_idx)
{
  if(robot_num_ == 0)
  {
    std::cout << "RobotPositionVisualizer::updateRobotPose :\n" <<
      "robot_num_ is 0!\n";

    return false;
  }

  if(robot_pose_vec_.size() != robot_num_)
  {
    robot_pose_vec_.resize(robot_num_);
  }

  if(robot_idx >= robot_num_)
  {
    std::cout << "RobotPositionVisualizer::updateRobotPose :\n" <<
      "robot_idx out of range!\n";

    return false;
  }

  gazebo_msgs::GetModelState get_model_state;
  get_model_state.request.model_name = robot_name_ + std::to_string(robot_idx);
  if(!get_model_state_client_.call(get_model_state))
  {
    std::cout << "RobotPositionVisualizer::updateRobotPose :\n" <<
      "robot " << robot_idx << " call get_model_state failed!\n";

    return false;
  }

  std::deque<geometry_msgs::Pose>& robot_pose_history = robot_pose_vec_[robot_idx];

  robot_pose_history.emplace_back(get_model_state.response.pose);
  if(robot_pose_history.size() > history_save_num_)
  {
    robot_pose_history.pop_front();
  }

  return true;
}

