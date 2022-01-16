#include <ros/ros.h>
#include <iostream>

#include <pointcloud2_to_object_vec_converter/PC2ToOBJS.h>

int main(int argc, char** argv)
{
  const size_t sleep_duration = 120;

  ros::init(argc, argv, "try_PointCloud2ToObjectVecConverterServer");

  ros::NodeHandle nh;
  ros::ServiceClient try_pointcloud_to_objects_client =
    nh.serviceClient<pointcloud2_to_object_vec_converter::PC2ToOBJS>("pointcloud2_to_object_vec_converter/convert_pointcloud2_to_object_vec");

  ROS_INFO("Start call pointcloud_to_objects_server service...");

  ROS_INFO("Start wait pointcloud2_to_object_vec_converter_server...");
  pointcloud2_to_object_vec_converter::PC2ToOBJS first_get_object_vec_serve;
  while(!try_pointcloud_to_objects_client.call(first_get_object_vec_serve))
  {
    ros::Duration(sleep_duration).sleep();
  }

  while(true)
  {
    ROS_INFO("Start call pointcloud2_to_object_vec_converter_server...");
    pointcloud2_to_object_vec_converter::PC2ToOBJS new_get_object_vec_serve;

    if(!try_pointcloud_to_objects_client.call(new_get_object_vec_serve))
    {
      std::cout << "call pointcloud2_to_object_vec_converter_server failed!" << std::endl;

      return 0;
    }

    const std::vector<sensor_msgs::PointCloud2>& objects = new_get_object_vec_serve.response.objects;
    std::cout << "get " << objects.size() << " objects\n";
    for(const sensor_msgs::PointCloud2& pointcloud2 : objects)
    {
      std::cout << "object size = " << pointcloud2.data.size() << std::endl;
    }

    ROS_INFO("Get response!");

    ros::Duration(sleep_duration).sleep();
  }

  return 1;
}

