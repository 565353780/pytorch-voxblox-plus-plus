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

bool GSMTopicSync::addGaussNoise(
    sensor_msgs::Image& image,
    const double& mu,
    const double& sigma,
    const double& noise_exp,
    const double& noise_max)
{
  std::random_device rd{};
  std::mt19937 gen{rd()};

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

      std::normal_distribution<> dist{mu, unit_noise_weight * sigma};
      const float current_gauss_noise = dist(gen);

      const float current_image_value = cv_image.at<float>(j, i);

      float new_image_value = std::fmax(
          0.0,
          current_image_value + noise_max * current_gauss_noise);
      new_image_value = std::fmin(
          new_image_value, std::numeric_limits<float>::max());

      cv_image.at<float>(j, i) = new_image_value;
    }
  }

  sensor_msgs::Image new_image = CvToRos(cv_image);
  image.data = new_image.data;
  return true;
}

bool GSMTopicSync::addPaperGaussNoise(
    sensor_msgs::Image& image,
    const double& noise_level)
{
  std::random_device rd{};
  std::mt19937 gen{rd()};

  std::normal_distribution<double> dist_1{0, 0.25 * noise_level};
  std::normal_distribution<double> dist_2{0, 1.0 / 36};

  cv::Mat cv_image = RosToCv(image);

  for(size_t i = 0; i < cv_image.cols; ++i)
  {
    for(size_t j = 0; j < cv_image.rows; ++j)
    {
      const double n_i = dist_1(gen);
      const double n_j = dist_1(gen);
      const double noise_d = dist_2(gen);

      size_t refer_col = size_t(i + n_i);
      size_t refer_row = size_t(j + n_j);
      refer_col = std::fmax(0, refer_col);
      refer_col = std::fmin(cv_image.cols - 1, refer_col);
      refer_row = std::fmax(0, refer_row);
      refer_row = std::fmin(cv_image.rows - 1, refer_row);

      const float refer_value = cv_image.at<float>(refer_row, refer_col);

      const float super_value = 35130.0;

      float new_image_value = super_value / (super_value / refer_value + noise_d + 0.5);

      new_image_value = std::fmax(0.0, new_image_value);
      new_image_value = std::fmin(new_image_value, std::numeric_limits<float>::max());

      cv_image.at<float>(j, i) = new_image_value;
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

  // const double min_noise_sigma = 0.01;
  // const double max_noise_sigma = 0.03;
  // addGaussNoise(camera_depth_image_raw_copy, 0.0, min_noise_sigma, 2, 1);

  addPaperGaussNoise(camera_depth_image_raw_copy, 1);

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

