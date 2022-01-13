#include <ros/ros.h>
#include <iostream>

#include "view_point_extractor/PC2ToViewPointVec.h"

#include "ViewPointSaver.h"

int main(int argc, char** argv)
{
  // ObjectSaver object_saver_;
  //
  // const float view_point_diff_max = 2;
  // const size_t view_point_similar_time = 3;
  // const size_t object_disappear_count_max = 2;
  //
  // for(size_t j = 0; j < 9; ++j)
  // {
  //   std::cout << "Start Search Valid ViewPoints..." << std::endl;
  //   object_saver_.resetObjectHistoryMatchState();
  //
  //   std::cout << "Start search..." << std::endl;
  //   for(size_t i = 0; i < 10 - j; ++i)
  //   {
  //     BBox3D object_bbox;
  //     object_bbox.setPosition(i, i, i, i + 1, i + 1, i + 1);
  //     ViewPoint3D object_view_point;
  //     object_view_point.setPosition(i, i, i, i + 1, i + 1, i + 1);
  //
  //     if(!object_saver_.isObjectViewPointVaild(
  //           i,
  //           object_bbox,
  //           object_view_point,
  //           view_point_diff_max,
  //           view_point_similar_time))
  //     {
  //       std::cout << "object " << i << "'s viewpoint not valid!" << std::endl;
  //     }
  //   }
  //
  //   std::cout << "Finish search!" << std::endl;
  //
  //   object_saver_.updateObjectHistory(object_disappear_count_max);
  //   std::cout << "Finish Search Valid ViewPoints..." << std::endl;
  // }
  // object_saver_.outputInfo(0);
  // return 0;

  ros::init(argc, argv, "try_ViewPointExtractorServer");

  ros::NodeHandle nh;
  ros::ServiceClient try_view_point_extractor_client =
      nh.serviceClient<view_point_extractor::PC2ToViewPointVec>("view_point_extractor/get_view_point_vec");

  std::cout << "Start call view_point_extractor_server service..." << std::endl;

  view_point_extractor::PC2ToViewPointVec get_view_point_vec_serve;
  if (!try_view_point_extractor_client.call(get_view_point_vec_serve))
  {
    std::cout << "call view_point_extractor_server failed!" << std::endl;

    return -1;
  }

  std::cout << "Get view_point_extractor_server response!" << std::endl;

  return 0;
}

