#include "global_segment_map_node/RobotPositionLoader.h"

bool RobotPositionLoader::reset()
{
  robot_bbox_vec_.clear();

  return true;
}

bool RobotPositionLoader::updateRobotBBoxVec(
    const std::string &world_name,
    const std::vector<std::string> &robot_name_vec)
{
  robot_bbox_vec_.clear();

  if(robot_name_vec.size() == 0)
  {
    std::cout << "RobotPositionLoader::updateRobotBBoxVec : " << std::endl <<
      "Input :\n" <<
      "\trobot_num = " << robot_name_vec.size() << std::endl <<
      "robot num must > 0!" << std::endl;

    return false;
  }

  std::vector<tf::StampedTransform> robot_tf_vec;
  robot_tf_vec.resize(robot_name_vec.size());

  for(size_t i = 0; i < robot_tf_vec.size(); ++i)
  {
    tf_listener_.waitForTransform(
        robot_name_vec[i], world_name,
        ros::Time(0), ros::Duration(10.0));

    tf_listener_.lookupTransform(
        robot_name_vec[i], world_name,
        ros::Time(0),
        robot_tf_vec[i]);
  }

  return true;
}

bool RobotPositionLoader::getRobotBBoxVec(
    const std::string &world_name,
    const std::vector<std::string> &robot_name_vec,
    std::vector<BBox3D> &robot_bbox_vec)
{
  robot_bbox_vec.clear();

  if(robot_name_vec.size() == 0)
  {
    std::cout << "RobotPositionLoader::getRobotBBoxVec : " << std::endl <<
      "Input :\n" <<
      "\trobot_num = " << robot_name_vec.size() << std::endl <<
      "robot num must > 0!" << std::endl;

    return false;
  }

  if(!updateRobotBBoxVec(world_name, robot_name_vec))
  {
    std::cout << "RobotPositionLoader::getRobotBBoxVec : " << std::endl <<
      "Input :\n" <<
      "\trobot_num = " << robot_name_vec.size() << std::endl <<
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

  for(const BBox3D &robot_bbox : robot_bbox_vec_)
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
    const BBox3D &bbox)
{
  if(x < bbox.x_min || x > bbox.x_max ||
      y < bbox.y_min || y > bbox.y_max ||
      z < bbox.z_min || z > bbox.z_max)
  {
    return false;
  }

  return true;
}

