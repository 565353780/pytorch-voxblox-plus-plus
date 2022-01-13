#include "robot_position_visualizer/RobotPositionVisualizer.h"
#include <cstdlib>
#include <ros/init.h>

int main(int argc, char** argv)
{
  std::cout << "Success run RobotPositionVisualizer!" << std::endl;

  ros::init(argc, argv, "RobotPositionVisualizer");

  RobotPositionVisualizer robot_position_visualizer;

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

  std::cout << "world_name = " << world_name << std::endl <<
    "robot_num = " << robot_num << std::endl <<
    "robot_name = " << robot_name << std::endl;

  robot_position_visualizer.setWorldAndRobotParam(world_name, robot_num, robot_name);

  return 1;
}

