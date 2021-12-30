#include "GSMTopicSync.h"

bool GSMTopicSync::setPubTopic()
{
  camera_depth_camera_info_pub_ = nh.advertise<CameraInfo>("camera/depth/camera_info", pub_rate_);
  camera_depth_image_raw_pub_ = nh.advertise<Image>("camera/depth/image_raw", pub_rate_);
  camera_rgb_camera_info_pub_ = nh.advertise<CameraInfo>("camera/rgb/camera_info", pub_rate_);
  camera_rgb_image_raw_pub_ = nh.advertise<Image>("camera/rgb/image_raw", pub_rate_);

  return true;
}

bool GSMTopicSync::setRobotParam(
    const std::string &world_name,
    const std::string &robot_name,
    const size_t &robot_num,
    const std::string& robot_depth_image_topic_prefix,
    const std::string& robot_rgb_image_topic_prefix,
    const std::string& robot_camera_groud_truth_topic_name)
{
  world_name_ = world_name;
  robot_name_ = robot_name;
  robot_num_ = robot_num;
  robot_depth_image_topic_prefix_ = robot_depth_image_topic_prefix;
  robot_rgb_image_topic_prefix_ = robot_rgb_image_topic_prefix;
  robot_camera_groud_truth_topic_name_ = robot_camera_groud_truth_topic_name;

  return true;
}

bool GSMTopicSync::startSync()
{
  //0 : ApproximateTime
  //1 : TimeSynchronizer
  size_t sync_mode = 1;

  switch(sync_mode)
  {
    case 0:
    {
      typedef sync_policies::ApproximateTime<CameraInfo, Image, CameraInfo, Image, Odometry> MySyncPolicy;

      std::vector<std::unique_ptr<message_filters::Subscriber<CameraInfo>>> camera_depth_camera_info_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<Image>>> camera_depth_image_raw_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<CameraInfo>>> camera_rgb_camera_info_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<Image>>> camera_rgb_image_raw_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<Odometry>>> camera_ground_truth_sub_vec;

      std::vector<std::unique_ptr<Synchronizer<MySyncPolicy>>> sync_vec;

      for(size_t robot_idx = 0; robot_idx < robot_num_; ++robot_idx)
      {
        camera_depth_camera_info_sub_vec.emplace_back(new message_filters::Subscriber<CameraInfo>(
              nh, robot_name_ + std::to_string(robot_idx) + robot_depth_image_topic_prefix_ + "camera_info", sub_queue_size_));
        camera_depth_image_raw_sub_vec.emplace_back(new message_filters::Subscriber<Image>(
              nh, robot_name_ + std::to_string(robot_idx) + robot_depth_image_topic_prefix_ + "image_raw", sub_queue_size_));
        camera_rgb_camera_info_sub_vec.emplace_back(new message_filters::Subscriber<CameraInfo>(
              nh, robot_name_ + std::to_string(robot_idx) + robot_rgb_image_topic_prefix_ + "camera_info", sub_queue_size_));
        camera_rgb_image_raw_sub_vec.emplace_back(new message_filters::Subscriber<Image>(
              nh, robot_name_ + std::to_string(robot_idx) + robot_rgb_image_topic_prefix_ + "image_raw", sub_queue_size_));
        camera_ground_truth_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
              nh, robot_name_ + std::to_string(robot_idx) + "/" + robot_camera_groud_truth_topic_name_, sub_queue_size_));

        sync_vec.emplace_back(new Synchronizer<MySyncPolicy>(MySyncPolicy(sub_queue_size_),
                                                             *camera_depth_camera_info_sub_vec[robot_idx],
                                                             *camera_depth_image_raw_sub_vec[robot_idx],
                                                             *camera_rgb_camera_info_sub_vec[robot_idx],
                                                             *camera_rgb_image_raw_sub_vec[robot_idx],
                                                             *camera_ground_truth_sub_vec[robot_idx]));
        
        sync_vec[robot_idx]->registerCallback(boost::bind(
              &GSMTopicSync::unionCallback, this, _1, _2, _3, _4, _5, robot_idx));
      }

      ros::spin();

      break;
    }
    case 1:
    {
      typedef TimeSynchronizer<CameraInfo, Image, CameraInfo, Image, Odometry> MySyncPolicy;

      std::vector<std::unique_ptr<message_filters::Subscriber<CameraInfo>>> camera_depth_camera_info_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<Image>>> camera_depth_image_raw_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<CameraInfo>>> camera_rgb_camera_info_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<Image>>> camera_rgb_image_raw_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<Odometry>>> camera_ground_truth_sub_vec;

      std::vector<std::unique_ptr<MySyncPolicy>> sync_vec;

      for(size_t robot_idx = 0; robot_idx < robot_num_; ++robot_idx)
      {
        camera_depth_camera_info_sub_vec.emplace_back(new message_filters::Subscriber<CameraInfo>(
              nh, robot_name_ + std::to_string(robot_idx) + robot_depth_image_topic_prefix_ + "camera_info", sub_queue_size_));
        camera_depth_image_raw_sub_vec.emplace_back(new message_filters::Subscriber<Image>(
              nh, robot_name_ + std::to_string(robot_idx) + robot_depth_image_topic_prefix_ + "image_raw", sub_queue_size_));
        camera_rgb_camera_info_sub_vec.emplace_back(new message_filters::Subscriber<CameraInfo>(
              nh, robot_name_ + std::to_string(robot_idx) + robot_rgb_image_topic_prefix_ + "camera_info", sub_queue_size_));
        camera_rgb_image_raw_sub_vec.emplace_back(new message_filters::Subscriber<Image>(
              nh, robot_name_ + std::to_string(robot_idx) + robot_rgb_image_topic_prefix_ + "image_raw", sub_queue_size_));
        camera_ground_truth_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
              nh, robot_name_ + std::to_string(robot_idx) + "/" + robot_camera_groud_truth_topic_name_, sub_queue_size_));
        
        sync_vec.emplace_back(new MySyncPolicy(*camera_depth_camera_info_sub_vec[robot_idx],
                                               *camera_depth_image_raw_sub_vec[robot_idx],
                                               *camera_rgb_camera_info_sub_vec[robot_idx],
                                               *camera_rgb_image_raw_sub_vec[robot_idx],
                                               *camera_ground_truth_sub_vec[robot_idx],
                                               sub_queue_size_));
        
        sync_vec[robot_idx]->registerCallback(boost::bind(
              &GSMTopicSync::unionCallback, this, _1, _2, _3, _4, _5, robot_idx));
      }
      
      ros::spin();

      break;
    }
  }

  return true;
}

void GSMTopicSync::unionCallback(
    const CameraInfoConstPtr& camera_depth_camera_info,
    const ImageConstPtr& camera_depth_image_raw,
    const CameraInfoConstPtr& camera_rgb_camera_info,
    const ImageConstPtr& camera_rgb_image_raw,
    const OdometryConstPtr& camera_ground_truth,
    size_t robot_idx)
{
  sensor_msgs::CameraInfo camera_depth_camera_info_copy = *camera_depth_camera_info;
  sensor_msgs::Image camera_depth_image_raw_copy = *camera_depth_image_raw;
  sensor_msgs::CameraInfo camera_rgb_camera_info_copy = *camera_rgb_camera_info;
  sensor_msgs::Image camera_rgb_image_raw_copy = *camera_rgb_image_raw;

  ros::Time current_time = camera_ground_truth->header.stamp;
  camera_depth_camera_info_copy.header.stamp = current_time;
  camera_depth_image_raw_copy.header.stamp = current_time;
  camera_rgb_camera_info_copy.header.stamp = current_time;
  camera_rgb_image_raw_copy.header.stamp = current_time;

  // double camera_position_x = camera_ground_truth->pose.pose.position.x;
  // double camera_position_y = camera_ground_truth->pose.pose.position.y;
  // double camera_position_z = camera_ground_truth->pose.pose.position.z;

  // tf2::Quaternion q_map_to_baselink;
  // tf2::convert(camera_ground_truth->pose.pose.orientation, q_map_to_baselink);

  // geometry_msgs::TransformStamped transformStamped_map_to_baselink;
  // transformStamped_map_to_baselink.header.frame_id = world_name_;
  // transformStamped_map_to_baselink.child_frame_id = robot_name_ + std::to_string(robot_idx) + "/base_link";
  // transformStamped_map_to_baselink.transform.translation.x = camera_position_x;
  // transformStamped_map_to_baselink.transform.translation.y = camera_position_y;
  // transformStamped_map_to_baselink.transform.translation.z = camera_position_z;
  // transformStamped_map_to_baselink.transform.rotation.x = q_map_to_baselink.x();
  // transformStamped_map_to_baselink.transform.rotation.y = q_map_to_baselink.y();
  // transformStamped_map_to_baselink.transform.rotation.z = q_map_to_baselink.z();
  // transformStamped_map_to_baselink.transform.rotation.w = q_map_to_baselink.w();
  // transformStamped_map_to_baselink.header.stamp = current_time;
  // tf_pub_.sendTransform(transformStamped_map_to_baselink);

  // tf2::Quaternion q_baselink_to_camera;
  // q_baselink_to_camera.setEuler(PI / 2.0, 0, -PI / 2.0);

  // geometry_msgs::TransformStamped transformStamped_baselink_to_camera;
  // transformStamped_baselink_to_camera.header.frame_id = robot_name_ + std::to_string(robot_idx);
  // transformStamped_baselink_to_camera.child_frame_id = robot_name_ + "frame_" + std::to_string(robot_idx);
  // transformStamped_baselink_to_camera.transform.translation.x = 0;
  // transformStamped_baselink_to_camera.transform.translation.y = 0;
  // transformStamped_baselink_to_camera.transform.translation.z = 0;
  // transformStamped_baselink_to_camera.transform.rotation.x = q_baselink_to_camera.x();
  // transformStamped_baselink_to_camera.transform.rotation.y = q_baselink_to_camera.y();
  // transformStamped_baselink_to_camera.transform.rotation.z = q_baselink_to_camera.z();
  // transformStamped_baselink_to_camera.transform.rotation.w = q_baselink_to_camera.w();
  // transformStamped_baselink_to_camera.header.stamp = current_time;
  // tf_pub_.sendTransform(transformStamped_baselink_to_camera);

  if(current_time == last_pub_camera_data_time_)
  {
    return;
  }
  camera_depth_camera_info_pub_.publish(camera_depth_camera_info_copy);
  camera_depth_image_raw_pub_.publish(camera_depth_image_raw_copy);
  camera_rgb_camera_info_pub_.publish(camera_rgb_camera_info_copy);
  camera_rgb_image_raw_pub_.publish(camera_rgb_image_raw_copy);

  // const std::string output_msg = "unionCallback : I heard srobot" + std::to_string(robot_idx) + "'s sync msgs.";
  // ROS_INFO(output_msg.c_str());
}

