#include "GSMTopicSync.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gsm_topic_sync");

  GSMTopicSync gsm_topic_sync;

  std::string world_name = "";
  std::string robot_name = "";
  size_t robot_num = 0;

  if(argc > 1)
  {
    world_name = argv[1];
  }

  if(argc > 2)
  {
    robot_name = argv[2];
  }

  if(argc > 3)
  {
    robot_num = atoi(argv[3]);
  }

  if(world_name == "" || robot_name == "" || robot_num == 0)
  {
    std::cout << "input not valid!" << std::endl;

    return -1;
  }

  gsm_topic_sync.setRobotParam(world_name, robot_name, robot_num);

  gsm_topic_sync.startSync();

  return 0;
}

