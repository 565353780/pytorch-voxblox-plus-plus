#include <ros/ros.h>
#include <iostream>

#include <point_state_manager/PointVecToStateVec.h>

bool tryAddNewPointVec()
{
  ros::NodeHandle nh;
  ros::ServiceClient try_add_new_point_vec_client =
      nh.serviceClient<point_state_manager::PointVecToStateVec>(
          "point_state_manager/add_new_point_vec");

  point_state_manager::PointVecToStateVec add_new_point_vec_serve;

  std::vector<geometry_msgs::Point> new_point_vec;
  geometry_msgs::Point new_point;

  new_point.x = 0;
  new_point.y = 1;
  new_point.z = 0;
  new_point_vec.emplace_back(new_point);

  new_point.x = 1;
  new_point.y = 1;
  new_point.z = 0;
  new_point_vec.emplace_back(new_point);

  add_new_point_vec_serve.request.point_vec = new_point_vec;

  if(!try_add_new_point_vec_client.call(add_new_point_vec_serve))
  {
    std::cout << "[ERROR][try_ViewPointExtractorServer::tryAddNewPointVec]\n" <<
      "\t call point_state_manager/add_new_point_vec failed!\n";

    return false;
  }

  std::cout << "[INFO][try_ViewPointExtractorServer::tryAddNewPointVec]\n" <<
    "\t call point_state_manager/add_new_point_vec success!\n";

  return true;
}

bool tryAddFinishPointVec()
{
  ros::NodeHandle nh;
  ros::ServiceClient try_add_finish_point_vec_client =
      nh.serviceClient<point_state_manager::PointVecToStateVec>(
          "point_state_manager/add_finish_point_vec");

  point_state_manager::PointVecToStateVec add_finish_point_vec_serve;

  std::vector<geometry_msgs::Point> finish_point_vec;

  geometry_msgs::Point finish_point;

  finish_point.x = 1;
  finish_point.y = 1;
  finish_point.z = 0;
  finish_point_vec.emplace_back(finish_point);

  finish_point.x = 2;
  finish_point.y = 1;
  finish_point.z = 0;
  finish_point_vec.emplace_back(finish_point);

  add_finish_point_vec_serve.request.point_vec = finish_point_vec;

  if(!try_add_finish_point_vec_client.call(add_finish_point_vec_serve))
  {
    std::cout << "[ERROR][try_ViewPointExtractorServer::tryAddFinishPointVec]\n" <<
      "\t call point_state_manager/add_finish_point_vec failed!\n";

    return false;
  }

  std::cout << "[INFO][try_ViewPointExtractorServer::tryAddFinishPointVec]\n" <<
    "\t call point_state_manager/add_finish_point_vec success!\n";

  return true;
}

bool tryGetPointStateVec()
{
  ros::NodeHandle nh;
  ros::ServiceClient try_get_point_state_vec_client =
      nh.serviceClient<point_state_manager::PointVecToStateVec>(
          "point_state_manager/get_point_state_vec");

  point_state_manager::PointVecToStateVec get_point_state_vec_serve;

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

  get_point_state_vec_serve.request.point_vec = query_point_vec;

  if(!try_get_point_state_vec_client.call(get_point_state_vec_serve))
  {
    std::cout << "[ERROR][try_ViewPointExtractorServer::tryGetPointStateVec]\n" <<
      "\t call point_state_manager/get_point_state_vec failed!\n";

    return false;
  }

  std::vector<int> state_vec = get_point_state_vec_serve.response.state_vec;

  for(size_t i = 0; i < state_vec.size(); ++i)
  {
    std::cout << "state " << i << " = " << state_vec[i] << std::endl;
  }

  std::cout << "[INFO][try_ViewPointExtractorServer::tryGetPointStateVec]\n" <<
    "\t call point_state_manager/get_point_state_vec success!\n";

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "try_PointStateManagerServer");

  tryAddNewPointVec();
  tryAddFinishPointVec();
  tryGetPointStateVec();

  return 0;
}

