#ifndef POINTCLOUD2_TO_OBJECTVEC_CONVERTERSERVER_H
#define POINTCLOUD2_TO_OBJECTVEC_CONVERTERSERVER_H

#include <iostream>
#include <ctime>
#include <fstream>
#include <dirent.h>
#include <cstdlib>

#include "PointCloud2ToObjectVecConverter.h"

#include <pointcloud2_to_object_vec_converter/PointCloud2Vec.h>
#include <pointcloud2_to_object_vec_converter/PC2ToOBJS.h>

#include "tensorboard_logger_ros/ScalarToBool.h"

class PointCloud2ToObjectVecConverterServer
{
public:
  PointCloud2ToObjectVecConverterServer():
    get_map_client_(nh_.serviceClient<vpp_msgs::GetMap>("gsm_node/get_map")),
    pointcloud_to_objects_server_(nh_.advertiseService("pointcloud2_to_object_vec_converter/convert_pointcloud2_to_object_vec",
                                  &PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback, this)),
    objects_pub_(nh_.advertise<pointcloud2_to_object_vec_converter::PointCloud2Vec>("pointcloud2_to_object_vec_converter/object_vec", queue_size_)),
    tensorboard_logger_client_(nh_.serviceClient<tensorboard_logger_ros::ScalarToBool>("tensorboard_logger/log_scalar"))
  {
    std::string log_prefix =
      std::string(std::getenv("HOME")) +
      "/.ros/RUN_LOG/PointCloud2ToObjectVecConverterServer/";

    std::time_t now = time(0);
    tm* ltm = localtime(&now);
    std::string log_date = std::to_string(1900 + ltm->tm_year) + "_";
    log_date += std::to_string(1 + ltm->tm_mon) + "_";
    log_date += std::to_string(ltm->tm_mday) + "_";
    log_date += std::to_string(ltm->tm_hour) + "-";
    log_date += std::to_string(ltm->tm_min) + "-";
    log_date += std::to_string(ltm->tm_sec);

    log_prefix += log_date + "/";

    if(opendir(log_prefix.c_str()) == NULL)
    {
      system(("mkdir -p " + log_prefix).c_str());
    }

    log_prefix_ = log_prefix;
    save_scene_idx_ = 0;
    log_start_time_ = clock();
  }

private:
  bool getObjectsFromPointCloud2Callback(
      pointcloud2_to_object_vec_converter::PC2ToOBJS::Request &req,
      pointcloud2_to_object_vec_converter::PC2ToOBJS::Response &res);

  bool saveScene(
      const sensor_msgs::PointCloud2& scene);

  bool saveObjectVec(
      const std::vector<sensor_msgs::PointCloud2>& object_vec);

  bool logObjectScalar(
      const std::vector<sensor_msgs::PointCloud2>& object_vec);

  bool logTensorBoard(
      const std::string& name,
      const size_t& step,
      const float& value);

private:
  ros::NodeHandle nh_;
	uint32_t queue_size_ = 1;

  ros::ServiceClient get_map_client_;
  ros::ServiceServer pointcloud_to_objects_server_;
  ros::Publisher objects_pub_;

  PointCloud2ToObjectVecConverter pointcloud2_to_object_vec_converter_;

  std::string log_prefix_;
  clock_t log_start_time_;
  size_t save_scene_idx_;
  size_t log_step_;

  ros::ServiceClient tensorboard_logger_client_;
};
 
#endif // POINTCLOUD2_TO_OBJECTVEC_CONVERTERSERVER_H
