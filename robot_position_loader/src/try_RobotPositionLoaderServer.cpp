#include <ros/ros.h>
#include <iostream>

#include "robot_position_loader/GetRobotBBox3DVec.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "try_RobotPositionLoaderServer");

  ros::NodeHandle nh;
  ros::ServiceClient try_robot_position_loader_client =
      nh.serviceClient<robot_position_loader::GetRobotBBox3DVec>("robot_position_loader/get_robot_bbox_vec");

  std::cout << "Start call robot_position_loader_server service..." << std::endl;

  robot_position_loader::GetRobotBBox3DVec get_robot_bbox_vec_serve;
  if (!try_robot_position_loader_client.call(get_robot_bbox_vec_serve))
  {
    std::cout << "call robot_position_loader_server failed!" << std::endl;

    return -1;
  }

  std::cout << "Get robot_position_loader_server response!" << std::endl;

  std::vector<robot_position_loader::BBox3D> &robot_bbox_vec =
    get_robot_bbox_vec_serve.response.robot_bbox_vec;

  std::cout << "BBox Num = " << robot_bbox_vec.size() << std::endl;

  for(const robot_position_loader::BBox3D &robot_bbox : robot_bbox_vec)
  {
    std::cout << "[" << robot_bbox.x_min << "," << robot_bbox.x_max << "][" <<
      robot_bbox.y_min << "," << robot_bbox.y_max << "][" <<
      robot_bbox.z_min << "," << robot_bbox.z_max << "]" << std::endl;
  }

  return 0;
}
