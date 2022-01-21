#include "ViewPointExtractorServer.h"

bool ViewPointExtractorServer::getViewPointVecCallback(
    view_point_extractor::PC2ToViewPointVec::Request &req,
    view_point_extractor::PC2ToViewPointVec::Response &res)
{
  std::cout << "ViewPointExtractorServer::getViewPointVecCallback :\n" <<
    "Start ViewPointExtractor serve!\n";

  const clock_t start_clock = clock();

  std::vector<view_point_extractor::ViewPoint> view_point_vec;
  view_point_extractor_.getMultiView(view_point_vec);

  time_spend_ = (clock() - start_clock) / CLOCKS_PER_SEC;

  res.view_point_vec = view_point_vec;

  // view_point_vec_pub_.publish(view_point_vec);

  if(!logTensorBoard(
        "ViewPointExtractorServer/viewpoint_extract_time",
        log_idx_,
        time_spend_))
  {
    std::cout << "ViewPointExtractorServer::getViewPointVecCallback :\n" <<
      "logTensorBoard for viewpoint_extract_time failed!\n";

    return false;
  }

  if(!logTensorBoard(
        "ViewPointExtractorServer/viewpoint_num",
        log_idx_,
        view_point_vec.size()))
  {
    std::cout << "ViewPointExtractorServer::getViewPointVecCallback :\n" <<
      "logTensorBoard for viewpoint_num failed!\n";

    return false;
  }

  ++log_idx_;

  return true;
}

bool ViewPointExtractorServer::logTensorBoard(
    const std::string& name,
    const size_t& step,
    const float& value)
{
  tensorboard_logger_ros::ScalarToBool tensorboard_logger_serve;

  // RUN_LOG PARAM
  tensorboard_logger_serve.request.scalar.name = name;
  tensorboard_logger_serve.request.scalar.step = step;
  tensorboard_logger_serve.request.scalar.value = value;

  if (!tensorboard_logger_client_.call(tensorboard_logger_serve))
  {
    std::cout << "ViewPointExtractorServer::logTensorBoard :\n" <<
      "call tensorboard_logger_server failed!\n";

    return false;
  }

  if(!tensorboard_logger_serve.response.success)
  {
    std::cout << "ViewPointExtractorServer::logTensorBoard :\n" <<
      "tensorboard_logger_server log failed!\n";

    return false;
  }

  return true;
}

