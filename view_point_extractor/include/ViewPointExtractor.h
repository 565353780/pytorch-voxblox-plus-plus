#ifndef VIEW_POINT_EXTRACTOR_H
#define VIEW_POINT_EXTRACTOR_H

#include <grnet_detect/PC2ToPC2.h>
#include <pointcloud2_to_object_vec_converter/PC2ToOBJS.h>
#include <pointcloud2_to_object_vec_converter/PointCloud2Vec.h>

#define PCL_NO_PRECOMPILE
#include <pcl/register_point_struct.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "EasyPCLVisualizer.h"
#include "view_point_extractor/ViewPoint.h"
#include "farthest_sampling/farthest_sampling.h"
#include "ViewPointSaver.h"

class ViewPointExtractor
{
  // clang-format off
public:
  ViewPointExtractor():
    grnet_input_pointcloud_size_(2048),
    grnet_detector_client_(
        nh_.serviceClient<grnet_detect::PC2ToPC2>("grnet_detect/detect")),
    pointcloud_to_objects_client_(
        nh_.serviceClient<pointcloud2_to_object_vec_converter::PC2ToOBJS>("pointcloud2_to_object_vec_converter/convert_pointcloud2_to_object_vec"))
  {
    coco_names_vec_ = {
      "BG",
      "person", "bicycle", "car", "motorcycle", "airplane",
      "bus", "train", "truck", "boat", "traffic light",
      "fire hydrant", "stop sign", "parking meter", "bench", "bird",
      "cat", "dog", "horse", "sheep", "cow",
      "elephant", "bear", "zebra", "giraffe", "backpack",
      "umbrella", "handbag", "tie", "suitcase", "frisbee",
      "skis", "snowboard", "sports ball", "kite", "baseball bat",
      "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
      "wine glass", "cup", "fork", "knife", "spoon",
      "bowl", "banana", "apple", "sandwich", "orange",
      "broccoli", "carrot", "hot dog", "pizza", "donut",
      "cake", "chair", "couch", "potted plant", "bed",
      "dining table", "toilet", "tv", "laptop", "mouse",
      "remote", "keyboard", "cell phone", "microwave", "oven",
      "toaster", "sink", "refrigerator", "book", "clock",
      "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
  }

  // clang-format on
  bool getMultiView(std::vector<view_point_extractor::ViewPoint>& view_point_vec);

private:
  bool sortVecIndexByMoreToLess(const std::vector<size_t>& source_vec, std::vector<size_t>& sorted_index_vec);

  bool getObjects(std::vector<sensor_msgs::PointCloud2>& objects);

  bool transPointCloud2VecToPointCloudVec(const std::vector<sensor_msgs::PointCloud2>& point_cloud2_vec,
                                          std::vector<sensor_msgs::PointCloud>& point_cloud_vec);

  bool transPointCloudVecToPointCloud2Vec(const std::vector<sensor_msgs::PointCloud>& point_cloud_vec,
                                          std::vector<sensor_msgs::PointCloud2>& point_cloud2_vec);

  bool getSamplingPointCloud(const sensor_msgs::PointCloud& source_point_cloud,
                             sensor_msgs::PointCloud& sampling_point_cloud, const int& sampling_point_num);

  bool getSamplingPointCloudVec(const std::vector<sensor_msgs::PointCloud>& source_point_cloud_vec,
                                std::vector<sensor_msgs::PointCloud>& sampling_point_cloud_vec,
                                const size_t& sampling_point_num);

  bool getFullPointCloud2FromPartialPointCloud2(const sensor_msgs::PointCloud2& partial_point_cloud2,
                                                sensor_msgs::PointCloud2& full_point_cloud2);

  bool
  getFullPointCloud2VecFromPartialPointCloud2Vec(const std::vector<sensor_msgs::PointCloud2>& partial_point_cloud2_vec,
                                                 std::vector<sensor_msgs::PointCloud2>& full_point_cloud2_vec);

  bool getMaxDistFullPointPosition(const sensor_msgs::PointCloud& partial_point_cloud,
                                   const sensor_msgs::PointCloud& full_point_cloud,
                                   view_point_extractor::ViewPoint& next_best_view_point);

  bool getNextBestViewPointPosition(const sensor_msgs::PointCloud& partial_point_cloud,
                                    const sensor_msgs::PointCloud& full_point_cloud,
                                    view_point_extractor::ViewPoint& next_best_view_point);

  bool getNextBestViewPointDirection(const sensor_msgs::PointCloud& point_cloud,
                                     view_point_extractor::ViewPoint& next_best_view_point);

  bool getNextBestViewPoint(const sensor_msgs::PointCloud& partial_point_cloud,
                            const sensor_msgs::PointCloud& full_point_cloud,
                            view_point_extractor::ViewPoint& next_best_view_point);

private:
  ros::NodeHandle nh_;

  std::vector<std::string> coco_names_vec_;

  int grnet_input_pointcloud_size_;

  ros::ServiceClient grnet_detector_client_;
  ros::ServiceClient pointcloud_to_objects_client_;

  EasyPCLVisualizer easy_pcl_visualzer_;

  ObjectSaver object_saver_;
};

#endif  // VIEW_POINT_EXTRACTOR_H
