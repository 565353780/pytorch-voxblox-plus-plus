#include "PointCloud2ToObjectVecConverter.h"

#include <pointcloud2_to_object_vec_converter/PointCloud2Vec.h>
#include <pointcloud2_to_object_vec_converter/PC2ToOBJS.h>

class PointCloud2ToObjectVecConverterServer
{
public:
  PointCloud2ToObjectVecConverterServer():
    get_map_client_(nh_.serviceClient<vpp_msgs::GetMap>("gsm_node/get_map")),
    pointcloud_to_objects_server_(nh_.advertiseService("pointcloud2_to_object_vec_converter/convert_pointcloud2_to_object_vec",
                                  &PointCloud2ToObjectVecConverterServer::getObjectsFromPointCloud2Callback, this)),
    objects_pub_(nh_.advertise<pointcloud2_to_object_vec_converter::PointCloud2Vec>("pointcloud2_to_object_vec_converter/object_vec", queue_size_))
  {
  }

private:
  bool getObjectsFromPointCloud2Callback(
      pointcloud2_to_object_vec_converter::PC2ToOBJS::Request &req,
      pointcloud2_to_object_vec_converter::PC2ToOBJS::Response &res);

  ros::NodeHandle nh_;

	uint32_t queue_size_ = 1;

  ros::ServiceClient get_map_client_;
  ros::ServiceServer pointcloud_to_objects_server_;
  ros::Publisher objects_pub_;

  PointCloud2ToObjectVecConverter pointcloud2_to_object_vec_converter_;
};
 
