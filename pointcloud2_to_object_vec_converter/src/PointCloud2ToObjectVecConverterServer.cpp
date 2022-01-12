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

  std::cout << "get map size: " << current_map_pointcloud.data.size() << std::endl;

  pointcloud2_to_object_vec_converter_.transPointCloud2ToObjects(current_map_pointcloud, objects);

  std::cout << "trans to objects size : " << objects.size() << std::endl;

  res.objects = objects;

  pointcloud2_to_object_vec_converter::PointCloud2Vec object_vec;

  object_vec.point_cloud2_vec = objects;

  // objects_pub_.publish(object_vec);

  return true;
}

