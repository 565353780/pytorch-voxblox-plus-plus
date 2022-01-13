#include "robot_position_loader/RobotPositionLoader.h"
#include <ros/init.h>

bool RobotPositionLoader::reset()
{
  robot_bbox_vec_.clear();

  return true;
}

bool RobotPositionLoader::setWorldAndRobotParam(
    const std::string &world_name,
    const size_t &robot_num,
    const std::string &robot_name)
{
  world_name_ = world_name;
  robot_num_ = robot_num;
  robot_name_ = robot_name;

  return true;
}

bool RobotPositionLoader::isPointInRobotBBox(
    const float &x,
    const float &y,
    const float &z)
{
  if(robot_bbox_vec_.size() == 0)
  {
    std::cout << "RobotPositionLoader::isPointInRobotBBox : " << std::endl <<
      "Input :\n" <<
      "\tpoint = [" << x << "," << y << "," << z << "]" << std::endl <<
      "robot_bbox_vec_ is empty!" << std::endl;

    return false;
  }

  for(const robot_position_loader::BBox3D &robot_bbox : robot_bbox_vec_)
  {
    if(isPointInBBox(x, y, z, robot_bbox))
    {
      return true;
    }
  }

  return false;
}

bool RobotPositionLoader::updateRobotPose()
{
  if(robot_num_ == 0)
  {
    std::cout << "RobotPositionLoader::updateRobotPose :\n" <<
      "robot_num_ is 0!\n";

    return false;
  }

  robot_pose_vec_.resize(robot_num_);

  for(size_t i = 0; i < robot_num_; ++i)
  {
    if(!updateRobotPose(i))
    {
      std::cout << "RobotPositionLoader::updateRobotPose :\n" <<
        "updateRobotPose for robot " << i << " failed!\n";

      return false;
    }
  }

  if(!updateRobotBBoxVec())
  {
      std::cout << "RobotPositionLoaderServer::updateRobotPose :\n" <<
        "updateRobotBBoxVec failed!\n";

      return false;
  }

  return true;
}

bool RobotPositionLoader::updateRobotPoseWithTimeStamp(
    const ros::Time& stamp)
{
  robot_bbox_vec_.resize(robot_num_);

  if(robot_num_ == 0)
  {
    std::cout << "RobotPositionLoader::updateRobotPoseWithTimeStamp : " << std::endl <<
      "robot_num_ is 0!" << std::endl;

    return false;
  }

  robot_pose_vec_.resize(robot_num_);

  for(size_t i = 0; i < robot_num_; ++i)
  {
    tf::StampedTransform current_tf;

    const std::string current_robot_name = robot_name_ + std::to_string(i) + "/base_link";

    try
    {
      tf_listener_.waitForTransform(
          world_name_, current_robot_name,
          stamp, ros::Duration(1.0));

      tf_listener_.lookupTransform(
          world_name_, current_robot_name,
          stamp,
          current_tf);

      std::deque<geometry_msgs::Pose>& robot_pose_history = robot_pose_vec_[i];

      geometry_msgs::Pose current_robot_pose;
      const tf::Vector3 &current_robot_position = current_tf.getOrigin();
      const tf::Quaternion& current_robot_orientation = current_tf.getRotation();
      current_robot_pose.position.x = current_robot_position.x();
      current_robot_pose.position.x = current_robot_position.y();
      current_robot_pose.position.x = current_robot_position.z();
      current_robot_pose.orientation.x = current_robot_orientation.x();
      current_robot_pose.orientation.y = current_robot_orientation.y();
      current_robot_pose.orientation.z = current_robot_orientation.z();
      current_robot_pose.orientation.w = current_robot_orientation.w();

      robot_pose_history.emplace_back(current_robot_pose);
      if(robot_pose_history.size() > history_save_num_)
      {
        robot_pose_history.pop_front();
      }
    }
    catch(...)
    {
      std::cout << "RobotPositionLoader::updateRobotPoseWithTimeStamp : " << std::endl <<
        "waitForTransform failed!" << std::endl;

      if(!updateRobotPose(i))
      {
        std::cout << "RobotPositionLoader::updateRobotPoseWithTimeStamp : " << std::endl <<
          "updateRobotPose for robot " << i << " failed!" << std::endl;

        return false;
      }
    }
  }

  if(!updateRobotBBoxVec())
  {
    std::cout << "RobotPositionLoader::updateRobotPoseWithTimeStamp : " << std::endl <<
      "updateRobotBBoxVec failed!" << std::endl;

    return false;
  }

  return true;
}

bool RobotPositionLoader::updateRobotPose(
    const size_t& robot_idx)
{
  if(robot_num_ == 0)
  {
    std::cout << "RobotPositionLoader::updateRobotPose :\n" <<
      "robot_num_ is 0!\n";

    return false;
  }

  if(robot_pose_vec_.size() != robot_num_)
  {
    robot_pose_vec_.resize(robot_num_);
  }

  if(robot_idx >= robot_num_)
  {
    std::cout << "RobotPositionLoader::updateRobotPose :\n" <<
      "robot_idx out of range!\n";

    return false;
  }

  gazebo_msgs::GetModelState get_model_state;
  get_model_state.request.model_name = robot_name_ + std::to_string(robot_idx);
  if(!get_model_state_client_.call(get_model_state))
  {
    std::cout << "RobotPositionLoader::updateRobotPose :\n" <<
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

bool RobotPositionLoader::updateRobotBBoxVec()
{
  const float x_down = -0.25;
  const float x_up = 0.25;
  const float y_down = -0.25;
  const float y_up = 0.25;
  const float z_down = -0.5;
  const float z_up = 0.5;

  robot_bbox_vec_.resize(robot_num_);

  if(robot_num_ == 0)
  {
    std::cout << "RobotPositionLoader::updateRobotBBoxVec : " << std::endl <<
      "robot_num_ is 0!" << std::endl;

    return false;
  }

  if(robot_pose_vec_.size() != robot_num_)
  {
    std::cout << "RobotPositionLoader::updateRobotBBoxVec : " << std::endl <<
      "robot_pose_vec.size() != robot_num_.size()!" << std::endl;

    return false;
  }

  for(size_t i = 0; i <robot_num_; ++i)
  {
    const std::deque<geometry_msgs::Pose>& robot_pose_history = robot_pose_vec_[i];

    for(const geometry_msgs::Pose& current_robot_pose : robot_pose_history)
    {
      robot_position_loader::BBox3D &current_robot_bbox =
        robot_bbox_vec_[i];
      current_robot_bbox.x_min = current_robot_pose.position.x + x_down;
      current_robot_bbox.x_max = current_robot_pose.position.x + x_up;
      current_robot_bbox.y_min = current_robot_pose.position.y + y_down;
      current_robot_bbox.y_max = current_robot_pose.position.y + y_up;
      current_robot_bbox.z_min = current_robot_pose.position.z + z_down;
      current_robot_bbox.z_max = current_robot_pose.position.z + z_up;

      // std::cout << "BBox for robot " << i << " :\n" <<
        // "[" << current_robot_bbox.x_min << "," << current_robot_bbox.x_max << "]" <<
        // "[" << current_robot_bbox.y_min << "," << current_robot_bbox.y_max << "]" <<
        // "[" << current_robot_bbox.z_min << "," << current_robot_bbox.z_max << "]\n";
    }
  }

  return true;
}

bool RobotPositionLoader::isPointInBBox(
    const float &x,
    const float &y,
    const float &z,
    const robot_position_loader::BBox3D &bbox)
{
  if(x < bbox.x_min || x > bbox.x_max ||
      y < bbox.y_min || y > bbox.y_max ||
      z < bbox.z_min || z > bbox.z_max)
  {
    return false;
  }

  return true;
}

