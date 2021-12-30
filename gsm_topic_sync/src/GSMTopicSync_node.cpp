#include "GSMTopicSync.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gsm_topic_sync");

  GSMTopicSync gsm_topic_sync;

  std::string world_name = "";
  std::string robot_name = "";
  size_t robot_num = 0;
  std::string camera_frame_topic_name = "";
  std::string camera_depth_image_topic_prefix = "";
  std::string camera_rgb_image_topic_prefix = "";
  std::string camera_groud_truth_topic_name = "";
  size_t pub_tf = 2;

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
    camera_frame_topic_name = argv[4];
  }

  if(argc > 5)
  {
    camera_depth_image_topic_prefix = argv[5];
  }

  if(argc > 6)
  {
    camera_rgb_image_topic_prefix = argv[6];
  }

  if(argc > 7)
  {
    camera_groud_truth_topic_name = argv[7];
  }

  if(argc > 8)
  {
    pub_tf = atoi(argv[8]);
  }

  if(world_name == "" ||
      robot_name == "" ||
      robot_num == 0 ||
      camera_frame_topic_name == "" ||
      camera_depth_image_topic_prefix == "" ||
      camera_rgb_image_topic_prefix == "" ||
      camera_groud_truth_topic_name == "" ||
      pub_tf == 2)
  {
    std::cout << "input not valid!" << std::endl;

    return -1;
  }

  gsm_topic_sync.setRobotParam(
      world_name,
      robot_name,
      robot_num,
      camera_frame_topic_name,
      camera_depth_image_topic_prefix,
      camera_rgb_image_topic_prefix,
      camera_groud_truth_topic_name,
      pub_tf);

  gsm_topic_sync.startSync();

  return 0;
}

