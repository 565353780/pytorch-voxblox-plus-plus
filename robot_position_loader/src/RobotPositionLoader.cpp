#include "robot_position_loader/RobotPositionLoader.h"

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

bool RobotPositionLoader::updateRobotBBoxVec()
{
  const float x_down = -0.1;
  const float x_up = 0.1;
  const float y_down = -0.1;
  const float y_up = 0.1;
  const float z_down = -0.1;
  const float z_up = 0.1;

  robot_bbox_vec_.resize(robot_num_);

  if(robot_num_ == 0)
  {
    std::cout << "RobotPositionLoader::updateRobotBBoxVec : " << std::endl <<
      "robot_num_ is 0!" << std::endl;

    return false;
  }

  for(size_t i = 0; i < robot_num_; ++i)
  {
    tf::StampedTransform current_tf;

    const std::string current_robot_name = robot_name_ + std::to_string(i);

    tf_listener_.waitForTransform(
        world_name_, current_robot_name,
        ros::Time(0), ros::Duration(10.0));

    tf_listener_.lookupTransform(
        world_name_, current_robot_name,
        ros::Time(0),
        current_tf);

    const tf::Vector3 &current_robot_center = current_tf.getOrigin();
    const float &current_robot_center_x = current_robot_center.x();
    const float &current_robot_center_y = current_robot_center.y();
    const float &current_robot_center_z = current_robot_center.z();

    robot_position_loader::BBox3D &current_robot_bbox =
      robot_bbox_vec_[i];
    current_robot_bbox.x_min = current_robot_center_x + x_down;
    current_robot_bbox.x_max = current_robot_center_x + x_up;
    current_robot_bbox.y_min = current_robot_center_y + y_down;
    current_robot_bbox.y_max = current_robot_center_y + y_up;
    current_robot_bbox.z_min = current_robot_center_z + z_down;
    current_robot_bbox.z_max = current_robot_center_z + z_up;
  }

  return true;
}

bool RobotPositionLoader::getRobotBBoxVec(
    std::vector<robot_position_loader::BBox3D> &robot_bbox_vec)
{
  robot_bbox_vec.clear();

  if(robot_num_ == 0)
  {
    std::cout << "RobotPositionLoader::getRobotBBoxVec : " << std::endl <<
      "robot_num_ is 0!" << std::endl;

    return false;
  }

  if(!updateRobotBBoxVec())
  {
    std::cout << "RobotPositionLoader::getRobotBBoxVec : " << std::endl <<
      "updateRobotBBoxVec failed!" << std::endl;

    return false;
  }

  robot_bbox_vec = robot_bbox_vec_;

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

