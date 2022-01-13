#ifndef EASY_PCL_VISUALIZER_H
#define EASY_PCL_VISUALIZER_H

#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#define PCL_NO_PRECOMPILE
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/register_point_struct.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

class EasyPCLVisualizer
{
public:
  EasyPCLVisualizer()
  {
    coco_names_vec_ = {
      "BG",           "person",         "bicycle",    "car",           "motorcycle",    "airplane",     "bus",
      "train",        "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",    "parking meter",
      "bench",        "bird",           "cat",        "dog",           "horse",         "sheep",        "cow",
      "elephant",     "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",     "handbag",
      "tie",          "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball",  "kite",
      "baseball bat", "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",       "wine glass",
      "cup",          "fork",           "knife",      "spoon",         "bowl",          "banana",       "apple",
      "sandwich",     "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",        "donut",
      "cake",         "chair",          "couch",      "potted plant",  "bed",           "dining table", "toilet",
      "tv",           "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",   "microwave",
      "oven",         "toaster",        "sink",       "refrigerator",  "book",          "clock",        "vase",
      "scissors",     "teddy bear",     "hair drier", "toothbrush"
    };

    self_added_idx_ = 0;
  }

  bool createRandomColor(double& r, double& g, double& b);

  bool createVis();

  bool createViewPort(const double& x_min, const double& y_min, const double& x_max, const double& y_max,
                      const std::string& viewport_name);

  bool addPointCloud(const sensor_msgs::PointCloud& point_cloud, const std::string& point_cloud_name, const double& r,
                     const double& g, const double& b, const size_t& viewport_idx);

  bool addArrow(const pcl::PointXYZ& start_point, const pcl::PointXYZ& end_point, const std::string& arrow_name,
                const double& r, const double& g, const double& b, const size_t& viewport_idx);

  bool addText(const std::string& text, const int& x, const int& y, const int& size, const double& r, const double& g,
               const double& b, const std::string& text_name, const size_t& viewport_idx);

  bool addText3D(const std::string& text, const pcl::PointXYZ& position, const double& scale, const double& r,
                 const double& g, const double& b, const std::string& text_name, const size_t& viewport_idx);

  bool showVis();

private:
  std::vector<std::string> coco_names_vec_;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  int self_added_idx_;
  std::vector<int> viewport_id_vec_;
};

#endif  // EASY_PCL_VISUALIZER_H
