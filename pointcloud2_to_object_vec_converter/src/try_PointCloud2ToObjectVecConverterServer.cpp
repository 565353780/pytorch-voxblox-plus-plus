#include <ros/ros.h>
#include <iostream>

#include <pointcloud2_to_object_vec_converter/PC2ToOBJS.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "try_PointCloud2ToObjectVecConverterServer");

  ros::NodeHandle nh;
  ros::ServiceClient try_pointcloud_to_objects_client =
    nh.serviceClient<pointcloud2_to_object_vec_converter::PC2ToOBJS>("pointcloud2_to_object_vec_converter/convert_pointcloud2_to_object_vec");

  ROS_INFO("Start call pointcloud_to_objects_server service...");

  pointcloud2_to_object_vec_converter::PC2ToOBJS get_object_vec_serve;
  if(!try_pointcloud_to_objects_client.call(get_object_vec_serve))
  {
    std::cout << "call pointcloud2_to_object_vec_converter_server failed!" << std::endl;

    return -1;
  }

  std::cout << "Get pointcloud2_to_object_vec_converter_server response!" << std::endl;

  while(true)
  {
    ROS_INFO("Start call pointcloud2_to_object_vec_converter_server...");
    pointcloud2_to_object_vec_converter::PC2ToOBJS new_get_object_vec_serve;

    if(!try_pointcloud_to_objects_client.call(new_get_object_vec_serve))
    {
      std::cout << "call pointcloud2_to_object_vec_converter_server failed!" << std::endl;

      return -1;
    }

    ROS_INFO("Get response!");

    ros::Duration(1).sleep();
  }

  return 0;
}

