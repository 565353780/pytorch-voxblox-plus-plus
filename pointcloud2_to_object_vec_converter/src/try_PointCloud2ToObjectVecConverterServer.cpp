#include <ros/ros.h>
#include <iostream>

#include <pointcloud2_to_object_vec_converter/PC2ToOBJS.h>

int main(int argc, char** argv)
{
  // RUN_LOG PARAM
  const size_t sleep_duration = 10;

  ros::init(argc, argv, "try_PointCloud2ToObjectVecConverterServer");

  ros::NodeHandle nh;
  ros::ServiceClient try_pointcloud_to_objects_client =
    nh.serviceClient<pointcloud2_to_object_vec_converter::PC2ToOBJS>("pointcloud2_to_object_vec_converter/convert_pointcloud2_to_object_vec");

  std::cout << "[INFO][try_PointCloud2ToObjectVecConverterServer]\n" <<
    "Start call pointcloud_to_objects_server service...\n";

  std::cout << "[INFO][try_PointCloud2ToObjectVecConverterServer]\n" <<
    "Start wait pointcloud2_to_object_vec_converter_server...\n";

  pointcloud2_to_object_vec_converter::PC2ToOBJS first_get_object_vec_serve;
  while(!try_pointcloud_to_objects_client.call(first_get_object_vec_serve))
  {
    ros::Duration(sleep_duration).sleep();
  }

  bool is_output = false;

  while(true)
  {
    ros::Duration(sleep_duration).sleep();

    std::cout << "Start call pointcloud2_to_object_vec_converter_server...\n";
    pointcloud2_to_object_vec_converter::PC2ToOBJS new_get_object_vec_serve;


    if(!try_pointcloud_to_objects_client.call(new_get_object_vec_serve))
    {
      if(is_output)
      {
        continue;
      }

      std::cout << "[ERROR][try_PointCloud2ToObjectVecConverterServer]\n" <<
        "call pointcloud2_to_object_vec_converter_server failed! start retry...\n";
      is_output = true;

      continue;
      // return 0;
    }

    const std::vector<sensor_msgs::PointCloud2>& objects = new_get_object_vec_serve.response.objects;
    std::cout << "get " << objects.size() << " objects\n";

    is_output = false;
  }

  return 1;
}

