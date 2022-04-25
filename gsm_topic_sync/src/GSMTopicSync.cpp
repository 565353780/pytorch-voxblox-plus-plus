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
    const std::string& camera_frame_topic_name,
    const std::string& camera_depth_image_topic_prefix,
    const std::string& camera_rgb_image_topic_prefix,
    const std::string& camera_groud_truth_topic_name,
    const size_t& pub_tf)
{
  world_name_ = world_name;
  robot_name_ = robot_name;
  robot_num_ = robot_num;
  camera_frame_topic_name_ = camera_frame_topic_name;
  camera_depth_image_topic_prefix_ = camera_depth_image_topic_prefix;
  camera_rgb_image_topic_prefix_ = camera_rgb_image_topic_prefix;
  camera_groud_truth_topic_name_ = camera_groud_truth_topic_name;
  pub_tf_ = pub_tf;

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
              nh, robot_name_ + std::to_string(robot_idx) + camera_depth_image_topic_prefix_ + "camera_info", sub_queue_size_));
        camera_depth_image_raw_sub_vec.emplace_back(new message_filters::Subscriber<Image>(
              nh, robot_name_ + std::to_string(robot_idx) + camera_depth_image_topic_prefix_ + "image_raw", sub_queue_size_));
        camera_rgb_camera_info_sub_vec.emplace_back(new message_filters::Subscriber<CameraInfo>(
              nh, robot_name_ + std::to_string(robot_idx) + camera_rgb_image_topic_prefix_ + "camera_info", sub_queue_size_));
        camera_rgb_image_raw_sub_vec.emplace_back(new message_filters::Subscriber<Image>(
              nh, robot_name_ + std::to_string(robot_idx) + camera_rgb_image_topic_prefix_ + "image_raw", sub_queue_size_));
        camera_ground_truth_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
              nh, robot_name_ + std::to_string(robot_idx) + "/" + camera_groud_truth_topic_name_, sub_queue_size_));

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
              nh, robot_name_ + std::to_string(robot_idx) + camera_depth_image_topic_prefix_ + "camera_info", sub_queue_size_));
        camera_depth_image_raw_sub_vec.emplace_back(new message_filters::Subscriber<Image>(
              nh, robot_name_ + std::to_string(robot_idx) + camera_depth_image_topic_prefix_ + "image_raw", sub_queue_size_));
        camera_rgb_camera_info_sub_vec.emplace_back(new message_filters::Subscriber<CameraInfo>(
              nh, robot_name_ + std::to_string(robot_idx) + camera_rgb_image_topic_prefix_ + "camera_info", sub_queue_size_));
        camera_rgb_image_raw_sub_vec.emplace_back(new message_filters::Subscriber<Image>(
              nh, robot_name_ + std::to_string(robot_idx) + camera_rgb_image_topic_prefix_ + "image_raw", sub_queue_size_));
        camera_ground_truth_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
              nh, robot_name_ + std::to_string(robot_idx) + "/" + camera_groud_truth_topic_name_, sub_queue_size_));
        
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

double GSMTopicSync::getGaussNoise(
    const double& mu,
    const double& sigma)
{
  const double epsilon = std::numeric_limits<double>::min();

  double u1 = rand() * (1.0 / RAND_MAX);
  double u2 = rand() * (1.0 / RAND_MAX);
  while(u1 <= epsilon)
  {
    u1 = rand() * (1.0 / RAND_MAX);
    u2 = rand() * (1.0 / RAND_MAX);
  }

  double z;
  const int use_cos = rand() % 2;
  if(use_cos == 1)
  {
    z = sqrt(-2.0*log(u1))*cos(2 * M_PI*u2);
    return z * sigma + mu;
  }

  z = sqrt(-2.0*log(u1))*sin(2 * M_PI*u2);
  return z * sigma + mu;
}

bool GSMTopicSync::addGaussNoise(
    sensor_msgs::Image& image,
    const double& mu,
    const double& sigma,
    const double& noise_exp,
    const double& noise_max)
{
  cv::Mat cv_image = RosToCv(image);

  const float image_width_center = cv_image.cols / 2.0;
  const float image_height_center = cv_image.rows / 2.0;
  const float unit_divide =
    std::pow(cv_image.cols / 2.0, noise_exp) +
    std::pow(cv_image.rows / 2.0, noise_exp);
  for(size_t i = 0; i < cv_image.cols; ++i)
  {
    for(size_t j = 0; j < cv_image.rows; ++j)
    {
      const float width_diff = std::abs(image_width_center - i);
      const float height_diff = std::abs(image_height_center - j);

      const float unit_noise_weight =
        (std::pow(width_diff, noise_exp) + std::pow(height_diff, noise_exp)) /
        unit_divide;

      const float current_gauss_noise = getGaussNoise(mu, unit_noise_weight * sigma);

      const short current_image_value = cv_image.at<short>(j, i);

      short new_image_value = std::fmax(
          short(0),
          short(current_image_value + noise_max * current_gauss_noise));
      new_image_value = std::fmin(
          new_image_value, std::numeric_limits<short>::max());

      cv_image.at<short>(j, i) = new_image_value;
    }
  }

  sensor_msgs::Image new_image = CvToRos(cv_image);
  image.data = new_image.data;
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

  addGaussNoise(camera_depth_image_raw_copy, 0.0, 20.0, 2, 1);

  ros::Time current_time = camera_ground_truth->header.stamp;

  camera_depth_camera_info_copy.header.stamp = current_time;
  camera_depth_image_raw_copy.header.stamp = current_time;
  camera_rgb_camera_info_copy.header.stamp = current_time;
  camera_rgb_image_raw_copy.header.stamp = current_time;

  double camera_position_x = camera_ground_truth->pose.pose.position.x;
  double camera_position_y = camera_ground_truth->pose.pose.position.y;
  double camera_position_z = camera_ground_truth->pose.pose.position.z;

  tf2::Quaternion q_map_to_camera;
  tf2::convert(camera_ground_truth->pose.pose.orientation, q_map_to_camera);

  geometry_msgs::TransformStamped transformStamped_map_to_camera;
  transformStamped_map_to_camera.header.frame_id = world_name_;
  transformStamped_map_to_camera.child_frame_id = robot_name_ + std::to_string(robot_idx) + "/" + camera_frame_topic_name_;
  transformStamped_map_to_camera.transform.translation.x = camera_position_x;
  transformStamped_map_to_camera.transform.translation.y = camera_position_y;
  transformStamped_map_to_camera.transform.translation.z = camera_position_z;
  transformStamped_map_to_camera.transform.rotation.x = q_map_to_camera.x();
  transformStamped_map_to_camera.transform.rotation.y = q_map_to_camera.y();
  transformStamped_map_to_camera.transform.rotation.z = q_map_to_camera.z();
  transformStamped_map_to_camera.transform.rotation.w = q_map_to_camera.w();
  transformStamped_map_to_camera.header.stamp = current_time;

  tf2::Quaternion q_baselink_to_camera;
  q_baselink_to_camera.setEuler(PI / 2.0, 0, -PI / 2.0);

  geometry_msgs::TransformStamped transformStamped_baselink_to_camera;
  transformStamped_baselink_to_camera.header.frame_id = robot_name_ + std::to_string(robot_idx) + "/" + camera_frame_topic_name_;
  transformStamped_baselink_to_camera.child_frame_id = robot_name_ + std::to_string(robot_idx) + "/robot_camera_frame";
  transformStamped_baselink_to_camera.transform.translation.x = 0;
  transformStamped_baselink_to_camera.transform.translation.y = 0;
  transformStamped_baselink_to_camera.transform.translation.z = 0;
  transformStamped_baselink_to_camera.transform.rotation.x = q_baselink_to_camera.x();
  transformStamped_baselink_to_camera.transform.rotation.y = q_baselink_to_camera.y();
  transformStamped_baselink_to_camera.transform.rotation.z = q_baselink_to_camera.z();
  transformStamped_baselink_to_camera.transform.rotation.w = q_baselink_to_camera.w();
  transformStamped_baselink_to_camera.header.stamp = current_time;

  if(current_time == last_pub_camera_data_time_)
  {
    return;
  }
  last_pub_camera_data_time_ = current_time;
  if(pub_tf_ == 1)
  {
    tf_pub_.sendTransform(transformStamped_map_to_camera);

    if(robot_name_ == "kinect_camera_")
    {
      tf_pub_.sendTransform(transformStamped_baselink_to_camera);
    }
  }
  camera_depth_camera_info_pub_.publish(camera_depth_camera_info_copy);
  camera_depth_image_raw_pub_.publish(camera_depth_image_raw_copy);
  camera_rgb_camera_info_pub_.publish(camera_rgb_camera_info_copy);
  camera_rgb_image_raw_pub_.publish(camera_rgb_image_raw_copy);

  // const std::string output_msg = "unionCallback : I heard srobot" + std::to_string(robot_idx) + "'s sync msgs.";
  // ROS_INFO(output_msg.c_str());
}

