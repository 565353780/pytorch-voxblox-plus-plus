#include <ros/ros.h>
#include <iostream>

#include <point_state_manager/PointVecToStateVec.h>

bool setPointVecState(
    const std::vector<geometry_msgs::Point>& point_vec,
    const int& state,
    const double& effect_radius)
{
  ros::NodeHandle nh;
  ros::ServiceClient set_point_vec_state_client =
      nh.serviceClient<point_state_manager::PointVecToStateVec>(
          "point_state_manager/set_point_vec_state");

  point_state_manager::PointVecToStateVec set_point_vec_state_serve;

  set_point_vec_state_serve.request.point_vec = point_vec;
  set_point_vec_state_serve.request.state = state;
  set_point_vec_state_serve.request.effect_radius = effect_radius;

  if(!set_point_vec_state_client.call(set_point_vec_state_serve))
  {
    std::cout << "[ERROR][try_ViewPointExtractorServer::setPointVecState]\n" <<
      "\t call point_state_manager/set_point_vec_state failed!\n";

    return false;
  }

  std::cout << "[INFO][try_ViewPointExtractorServer::setPointVecState]\n" <<
    "\t call point_state_manager/set_point_vec_state success!\n";

  return true;
}

bool getPointStateVec(
    const std::vector<geometry_msgs::Point>& point_vec,
    std::vector<int>& state_vec)
{
  ros::NodeHandle nh;
  ros::ServiceClient get_point_state_vec_client =
      nh.serviceClient<point_state_manager::PointVecToStateVec>(
          "point_state_manager/get_point_state_vec");

  point_state_manager::PointVecToStateVec get_point_state_vec_serve;

  get_point_state_vec_serve.request.point_vec = point_vec;

  if(!get_point_state_vec_client.call(get_point_state_vec_serve))
  {
    std::cout << "[ERROR][try_ViewPointExtractorServer::getPointStateVec]\n" <<
      "\t call point_state_manager/get_point_state_vec failed!\n";

    return false;
  }

  state_vec = get_point_state_vec_serve.response.state_vec;

  std::cout << "[INFO][try_ViewPointExtractorServer::getPointStateVec]\n" <<
    "\t call point_state_manager/get_point_state_vec success!\n";

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "try_PointStateManagerServer");

  std::vector<geometry_msgs::Point> point_vec;
  geometry_msgs::Point new_point;

  new_point.x = 0;
  new_point.y = 1;
  new_point.z = 0;
  point_vec.emplace_back(new_point);

  new_point.x = 1;
  new_point.y = 1;
  new_point.z = 0;
  point_vec.emplace_back(new_point);

  setPointVecState(point_vec, 100, 0.2);

  point_vec[0].x = 2;

  setPointVecState(point_vec, 0, 0.2);

  std::vector<geometry_msgs::Point> query_point_vec;

  geometry_msgs::Point query_point;

  query_point.x = 0;
  query_point.y = 1;
  query_point.z = 0;
  query_point_vec.emplace_back(query_point);

  query_point.x = 1;
  query_point.y = 1;
  query_point.z = 0;
  query_point_vec.emplace_back(query_point);

  query_point.x = 2;
  query_point.y = 1;
  query_point.z = 0;
  query_point_vec.emplace_back(query_point);

  query_point.x = 3;
  query_point.y = 1;
  query_point.z = 0;
  query_point_vec.emplace_back(query_point);

  std::vector<int> state_vec;

  getPointStateVec(query_point_vec, state_vec);

  for(size_t i = 0; i < state_vec.size(); ++i)
  {
    std::cout << "state " << i << " = " << state_vec[i] << std::endl;
  }

  return 0;
}

