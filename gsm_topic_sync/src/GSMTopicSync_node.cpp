#include "GSMTopicSync.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gsm_topic_sync");

  GSMTopicSync gsm_topic_sync;

  std::string world_name = "";
  std::string robot_name = "";
  size_t robot_num = 0;
  std::string robot_depth_image_topic_prefix = "";
  std::string robot_rgb_image_topic_prefix = "";
  std::string robot_camera_groud_truth_topic_name = "";

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

  if(argc > 4)
  {
    robot_depth_image_topic_prefix = argv[4];
  }

  if(argc > 5)
  {
    robot_rgb_image_topic_prefix = argv[5];
  }

  if(argc > 6)
  {
    robot_camera_groud_truth_topic_name = argv[6];
  }

  if(world_name == "" ||
      robot_name == "" ||
      robot_num == 0 ||
      robot_depth_image_topic_prefix == "" ||
      robot_rgb_image_topic_prefix == "" ||
      robot_camera_groud_truth_topic_name == "")
  {
    std::cout << "input not valid!" << std::endl;

    return -1;
  }

  gsm_topic_sync.setRobotParam(
      world_name,
      robot_name,
      robot_num,
      robot_depth_image_topic_prefix,
      robot_rgb_image_topic_prefix,
      robot_camera_groud_truth_topic_name);

  gsm_topic_sync.startSync();

  return 0;
}

