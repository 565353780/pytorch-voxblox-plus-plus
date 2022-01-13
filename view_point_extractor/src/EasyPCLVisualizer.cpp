#include "EasyPCLVisualizer.h"

bool EasyPCLVisualizer::createRandomColor(
    double &r,
    double &g,
    double &b)
{
  r = std::rand() % 255;
  g = std::rand() % 255;
  b = std::rand() % 255;

  return true;
}

bool EasyPCLVisualizer::createVis()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  viewer_ = viewer;

  viewport_id_vec_.clear();
  self_added_idx_ = 0;

  return true;
}

bool EasyPCLVisualizer::createViewPort(
    const double &x_min,
    const double &y_min,
    const double &x_max,
    const double &y_max,
    const std::string &viewport_name)
{
  int viewport_id;
  viewer_->createViewPort(x_min, y_min, x_max, y_max, viewport_id);

  viewport_id_vec_.emplace_back(viewport_id);

  if(viewport_name == "")
  {
    return true;
  }

  addText(
      viewport_name,
      10, 10,
      30,
      0, 255, 0,
      "",
      viewport_id_vec_.size() - 1);

  return true;
}

bool EasyPCLVisualizer::addPointCloud(
    const sensor_msgs::PointCloud &point_cloud,
    const std::string &point_cloud_name,
    const double &r,
    const double &g,
    const double &b,
    const size_t &viewport_idx)
{
  std::string new_point_cloud_name = point_cloud_name;
  if(new_point_cloud_name == "")
  {
    new_point_cloud_name = "point_cloud_" + std::to_string(self_added_idx_);
    ++self_added_idx_;
  }

  sensor_msgs::PointCloud2 point_cloud2;
  sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(point_cloud2, *pcl_point_cloud);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pcl_point_cloud_color(pcl_point_cloud, r, g, b);
  viewer_->addPointCloud<pcl::PointXYZ>(
      pcl_point_cloud,
      pcl_point_cloud_color,
      new_point_cloud_name,
      viewport_id_vec_[viewport_idx]);
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      1,
      new_point_cloud_name,
      viewport_id_vec_[viewport_idx]);

  return true;
}

bool EasyPCLVisualizer::addArrow(
    const pcl::PointXYZ &start_point,
    const pcl::PointXYZ &end_point,
    const std::string &arrow_name,
    const double &r,
    const double &g,
    const double &b,
    const size_t &viewport_idx)
{
  std::string new_arrow_name = arrow_name;
  if(new_arrow_name == "")
  {
    new_arrow_name = "arrow_" + std::to_string(self_added_idx_);
    ++self_added_idx_;
  }

  viewer_->addArrow(
      end_point,
      start_point,
      r/255.0, g/255.0, b/255.0,
      false,
      new_arrow_name,
      viewport_id_vec_[viewport_idx]);

  return true;
}

bool EasyPCLVisualizer::addText(
    const std::string &text,
    const int &x,
    const int &y,
    const int &size,
    const double &r,
    const double &g,
    const double &b,
    const std::string &text_name,
    const size_t &viewport_idx)
{
  std::string new_text_name = text_name;
  if(new_text_name == "")
  {
    new_text_name = "text_" + std::to_string(self_added_idx_);
    ++self_added_idx_;
  }

  viewer_->addText(
      text,
      x, y,
      size,
      r/255.0, g/255.0, b/255.0,
      new_text_name, viewport_id_vec_[viewport_idx]);

  return true;
}

bool EasyPCLVisualizer::addText3D(
    const std::string &text,
    const pcl::PointXYZ &position,
    const double &scale,
    const double &r,
    const double &g,
    const double &b,
    const std::string &text_name,
    const size_t &viewport_idx)
{
  std::string new_text_name = text_name;
  if(new_text_name == "")
  {
    new_text_name = "text3d_" + std::to_string(self_added_idx_);
    ++self_added_idx_;
  }

  viewer_->addText3D(
      text,
      position,
      scale,
      r/255.0, g/255.0, b/255.0,
      new_text_name,
      viewport_id_vec_[viewport_idx]);

  return true;
}

bool EasyPCLVisualizer::showVis()
{
  while(!viewer_->wasStopped())
  {
    viewer_->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return true;
}

