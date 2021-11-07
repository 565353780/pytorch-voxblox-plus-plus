#include "robot_position_loader/RobotPositionLoaderServer.h"
#include <cstdlib>
#include <ros/init.h>

int main(int argc, char** argv)
{
  std::cout << "Success run RobotPositionLoaderServer!" << std::endl;

  ros::init(argc, argv, "RobotPositionLoaderServer");

  RobotPositionLoaderServer robot_position_loader_server;

  std::string world_name = "";
  size_t robot_num = 0;
  std::string robot_name = "";

  if(argc > 1)
  {
    world_name = argv[1];
  }

  if(argc > 2)
  {
    robot_num = atoi(argv[2]);
  }

  if(argc > 3)
  {
    robot_name = argv[3];
  }

  if(world_name == "" || robot_num == 0 || robot_name == "")
  {
    std::cout << "input not valid!" << std::endl;

    return -1;
  }

  robot_position_loader_server.setWorldAndRobotParam(world_name, robot_num, robot_name);

  ros::spin();

  return 1;
}
