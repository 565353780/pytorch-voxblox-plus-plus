#ifndef GSM_TOPIC_SYNC_H
#define GSM_TOPIC_SYNC_H

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ImageConvert.h>

#define PI 3.14159265358979

using namespace sensor_msgs;
using namespace message_filters;
using namespace nav_msgs;

class GSMTopicSync
{
public:
  GSMTopicSync()
  {
    sub_queue_size_ = 1;
    pub_rate_ = 100;

    setPubTopic();

    last_pub_camera_data_time_ = ros::Time();
  }

  bool setPubTopic();

  bool setRobotParam(
      const std::string& world_name,
      const std::string& robot_name,
      const size_t& robot_num,
      const std::string& camera_frame_topic_name,
      const std::string& camera_depth_image_topic_prefix,
      const std::string& camera_rgb_image_topic_prefix,
      const std::string& camera_groud_truth_topic_name,
      const size_t& pub_tf);

  bool startSync();

private:
  bool addGaussNoise(
      sensor_msgs::Image& image,
      const double& mu,
      const double& sigma,
      const double& noise_exp,
      const double& noise_max);

  bool addPaperGaussNoise(
      sensor_msgs::Image& image,
      const double& noise_level);

  void unionCallback(
      const CameraInfoConstPtr& camera_depth_camera_info,
      const ImageConstPtr& camera_depth_image_raw,
      const CameraInfoConstPtr& camera_rgb_camera_info,
      const ImageConstPtr& camera_rgb_image_raw,
      const OdometryConstPtr& camera_ground_truth,
      size_t robot_idx);

public:
    ros::NodeHandle nh;
    
private:
    size_t sub_queue_size_;
    size_t pub_rate_;
    
    ros::Publisher camera_depth_camera_info_pub_;
    ros::Publisher camera_depth_image_raw_pub_;
    ros::Publisher camera_rgb_camera_info_pub_;
    ros::Publisher camera_rgb_image_raw_pub_;
    tf2_ros::TransformBroadcaster tf_pub_;

    std::string world_name_;
    size_t robot_num_;
    std::string robot_name_;
    std::string camera_frame_topic_name_;
    std::string camera_depth_image_topic_prefix_;
    std::string camera_rgb_image_topic_prefix_;
    std::string camera_groud_truth_topic_name_;
    size_t pub_tf_;

    ros::Time last_pub_camera_data_time_;
};

#endif // GSM_TOPIC_SYNC_H

