#include "ViewPointExtractorServer.h"

bool ViewPointExtractorServer::getViewPointVecCallback(
    view_point_extractor::PC2ToViewPointVec::Request &req,
    view_point_extractor::PC2ToViewPointVec::Response &res)
{
  std::cout << "Start ViewPointExtractor serve!" << std::endl;

  std::vector<view_point_extractor::ViewPoint> view_point_vec;
  view_point_extractor_.getMultiView(view_point_vec);

  res.view_point_vec = view_point_vec;

  // view_point_vec_pub_.publish(view_point_vec);

  return true;
}

