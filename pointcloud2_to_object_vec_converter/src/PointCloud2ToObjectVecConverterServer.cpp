#include "PointCloud2ToObjectVecConverterServer.h"

bool PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback(
    pointcloud2_to_object_vec_converter::PC2ToOBJS::Request &req,
    pointcloud2_to_object_vec_converter::PC2ToOBJS::Response &res)
{
  ROS_INFO("Start pointcloud_to_objects serve!");

  vpp_msgs::GetMap get_map_request;

  if(!get_map_client_.call(get_map_request))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback : " <<
      "call get_map failed!" << std::endl;

    return false;
  }

  // objects[*].channels[*].name = [semantic_label]
  std::vector<sensor_msgs::PointCloud2> objects;

  const sensor_msgs::PointCloud2 &current_map_pointcloud = get_map_request.response.map_cloud;

  pointcloud2_to_object_vec_converter_.transPointCloud2ToObjects(current_map_pointcloud, objects);

  res.objects = objects;

  pointcloud2_to_object_vec_converter::PointCloud2Vec object_vec;

  object_vec.point_cloud2_vec = objects;

  // objects_pub_.publish(object_vec);

  if(!saveScene(current_map_pointcloud))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback : " <<
      "saveScene failed!" << std::endl;

    return false;
  }

  if(!saveObjectVec(objects))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback : " <<
      "saveObjectVec failed!" << std::endl;

    return false;
  }

  if(!logTensorBoard(objects.size()))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback : " <<
      "logTensorBoard failed!" << std::endl;

    return false;
  }

  ++log_idx_;

  return true;
}

bool PointCloud2ToObjectVecConverterServer::logTensorBoard(
    const size_t& object_num)
{
  tensorboard_logger_ros::ScalarToBool tensorboard_logger_serve;

  // RUN_LOG PARAM
  tensorboard_logger_serve.request.scalar.name =
    "PointCloud2ToObjectVecConverterServer/object_num";
  tensorboard_logger_serve.request.scalar.step = log_idx_;
  tensorboard_logger_serve.request.scalar.value = object_num;

  if (!tensorboard_logger_client_.call(tensorboard_logger_serve))
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::logTensorBoard :\n" <<
      "call tensorboard_logger_server failed!\n";

    return false;
  }

  if(!tensorboard_logger_serve.response.success)
  {
    std::cout << "PointCloud2ToObjectVecConverterServer::logTensorBoard :\n" <<
      "tensorboard_logger_server log failed!\n";

    return false;
  }

  return true;
}

bool PointCloud2ToObjectVecConverterServer::saveScene(
    const sensor_msgs::PointCloud2& scene)
{
  // RUN_LOG PARAM
  const size_t save_duration = 10;

  // scene_point_cloud.channels[*].name =
  // [distance, weight, segment_label, semantic_class, instance_label]
  const size_t semantic_class_channel_idx = 3;
  pcl::PointCloud<PointCloudWithSemanticAndInstanceLabel> pcl_point_cloud;

  sensor_msgs::PointCloud scene_point_cloud;
  convertPointCloud2ToPointCloud(scene, scene_point_cloud);

  pcl_point_cloud.width = scene_point_cloud.points.size();
  pcl_point_cloud.height = 1;

  pcl_point_cloud.points.resize(pcl_point_cloud.width * pcl_point_cloud.height);

  for(size_t i = 0; i < scene_point_cloud.points.size(); ++i)
  {
    const float &current_x = scene_point_cloud.points[i].x;
    const float &current_y = scene_point_cloud.points[i].y;
    const float &current_z = scene_point_cloud.points[i].z;
    const size_t &current_label =
      scene_point_cloud.channels[semantic_class_channel_idx].values[i];

    pcl_point_cloud.points[i].x = current_x;
    pcl_point_cloud.points[i].y = current_y;
    pcl_point_cloud.points[i].z = current_z;
    pcl_point_cloud.points[i].semantic_label = current_label;
  }

  if(log_idx_ % save_duration == 0)
  {
    pcl::io::savePCDFileASCII(
        log_prefix_ + "scene_" + std::to_string(log_idx_) + ".pcd",
        pcl_point_cloud);
  }

  return true;
}

bool PointCloud2ToObjectVecConverterServer::saveObjectVec(
    const std::vector<sensor_msgs::PointCloud2>& object_vec)
{
  if(object_vec.size() == 0)
  {
    return true;
  }

  system(("rm " + log_prefix_ + "/object*").c_str());

  for(size_t i = 0; i < object_vec.size(); ++i)
  {
    const sensor_msgs::PointCloud2& object = object_vec[i];
    pcl::PointCloud<PointCloudWithSemanticAndInstanceLabel> pcl_point_cloud;

    pcl::fromROSMsg(object, pcl_point_cloud);

    pcl::io::savePCDFileASCII(
        log_prefix_ + "object_" + std::to_string(i) + ".pcd",
        pcl_point_cloud);
  }

  return true;
}

