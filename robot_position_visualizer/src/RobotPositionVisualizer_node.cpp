#include "robot_position_visualizer/RobotPositionVisualizer.h"
#include <ros/init.h>

int main(int argc, char** argv)
{
  std::cout << "Success run RobotPositionVisualizer!" << std::endl;

  ros::init(argc, argv, "RobotPositionVisualizer");

  RobotPositionVisualizer robot_position_visualizer;

  return 1;
}

