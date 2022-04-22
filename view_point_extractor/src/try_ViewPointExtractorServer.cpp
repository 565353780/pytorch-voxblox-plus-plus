#include <ros/ros.h>
#include <iostream>

#include "view_point_extractor/PC2ToViewPointVec.h"

#include "ViewPointSaver.h"

int main(int argc, char** argv)
{
  // RUN_LOG PARAM
  const size_t sleep_duration = 20;

  ros::init(argc, argv, "try_ViewPointExtractorServer");

  ros::NodeHandle nh;
  ros::ServiceClient try_view_point_extractor_client =
      nh.serviceClient<view_point_extractor::PC2ToViewPointVec>("view_point_extractor/get_view_point_vec");

  // OUTPUT for try call get_view_point_vec
  // std::cout << "[INFO][try_ViewPointExtractorServer::main]\n" <<
    // "\t start wait pointcloud2_to_object_vec_converter_server...\n";

  view_point_extractor::PC2ToViewPointVec first_get_view_point_vec_serve;
  while(!try_view_point_extractor_client.call(first_get_view_point_vec_serve))
  {
    ros::Duration(sleep_duration).sleep();
  }

  while(true)
  {
    ros::Duration(sleep_duration).sleep();

    // std::cout << "[INFO][try_ViewPointExtractorServer::main]\n" <<
      // "\t start call view_point_extractor_server service...\n";

    view_point_extractor::PC2ToViewPointVec get_view_point_vec_serve;
    if (!try_view_point_extractor_client.call(get_view_point_vec_serve))
    {
      std::cout << "[ERROR][try_ViewPointExtractorServer::main]\n" <<
        "\t call view_point_extractor_server failed!\n";

      return -1;
    }

    // std::cout << "[INFO][try_ViewPointExtractorServer::main]\n" <<
      // "\t get response!\n";

  }
  return 0;
}

