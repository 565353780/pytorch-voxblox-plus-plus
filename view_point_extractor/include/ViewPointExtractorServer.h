#ifndef VIEW_POINT_EXTRACTOR_SERVER_H
#define VIEW_POINT_EXTRACTOR_SERVER_H

#include "ViewPointExtractor.h"
#include "view_point_extractor/ViewPointVec.h"
#include "view_point_extractor/PC2ToViewPointVec.h"

class ViewPointExtractorServer
{
public:
  ViewPointExtractorServer() :
    view_point_extractor_server_(nh_.advertiseService("view_point_extractor/get_view_point_vec",
                                 &ViewPointExtractorServer::getViewPointVecCallback, this)),
    view_point_vec_pub_(nh_.advertise<view_point_extractor::ViewPointVec>("view_point_extractor/view_point_vec", queue_size_))
  {
  }

private:
  bool getViewPointVecCallback(
      view_point_extractor::PC2ToViewPointVec::Request &req,
      view_point_extractor::PC2ToViewPointVec::Response &res);

private:
  ros::NodeHandle nh_;

  uint32_t queue_size_ = 1;
  ros::ServiceServer view_point_extractor_server_;
  ros::Publisher view_point_vec_pub_;

  ViewPointExtractor view_point_extractor_;
};

#endif //VIEW_POINT_EXTRACTOR_SERVER_H
