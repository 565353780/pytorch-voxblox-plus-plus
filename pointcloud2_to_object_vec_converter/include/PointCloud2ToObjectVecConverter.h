#ifndef POINT_CLOUD2_TO_OBJECT_VEC_CONVERTER_H
#define POINT_CLOUD2_TO_OBJECT_VEC_CONVERTER_H

#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <bits/stdint-uintn.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <vpp_msgs/GetMap.h>

#define PCL_NO_PRECOMPILE
#include <pcl/register_point_struct.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// create a new point type for using pcl to generate sensor_msgs::PointCloud2::data
struct PointWithRGBAndLabel
{
  PCL_ADD_POINT4D
  PCL_ADD_RGB

  float distance;
  float weight;
  std::uint16_t segment_label;
  std::uint8_t semantic_label;
  std::uint16_t instance_label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointWithRGBAndLabel,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (std::uint8_t, r, r)(std::uint8_t, g, g)(std::uint8_t, b, b)(std::uint8_t, a, a)
                                  (float, distance, distance)
                                  (float, weight, weight)
                                  (std::uint16_t, segment_label, segment_label)
                                  (std::uint8_t, semantic_label, semantic_label)
                                  (std::uint16_t, instance_label, instance_label))

struct BBox
{
  bool is_valid = false;

  float x_min = 0;
  float x_max = 0;
  float y_min = 0;
  float y_max = 0;
};

struct LabeledObject
{
  size_t instance_label = 0;

  std::vector<PointWithRGBAndLabel> points;

  bool reset();

  bool setLabel(
      const size_t &new_instance_label);

  bool addPoint(
      const float &point_x,
      const float &point_y,
      const float &point_z,
      const std::uint8_t &point_r,
      const std::uint8_t &point_g,
      const std::uint8_t &point_b,
      const std::uint8_t &point_a,
      const float &point_distance,
      const float &point_weight,
      const std::uint16_t &segment_label,
      const std::uint8_t &semantic_label);

  bool getBBox(
      BBox &bbox);

  float getDist(
      const float &x1,
      const float &y1,
      const float &z1,
      const float &x2,
      const float &y2,
      const float &z2);

  float getDistByIndex(
      const size_t &point_index,
      const float &point_x,
      const float &point_y,
      const float &point_z);

  float getMinDist(
      const float &point_x,
      const float &point_y,
      const float &point_z);
};

class PointCloud2ToObjectVecConverter
{
public:
  PointCloud2ToObjectVecConverter()
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

  bool transPointCloud2ToObjects(
      const sensor_msgs::PointCloud2 &point_cloud2,
      std::vector<sensor_msgs::PointCloud2> &objects);

private:
  bool isSamePoint(
      const geometry_msgs::Point32 &point1,
      const geometry_msgs::Point32 &point2);

  bool splitPointCloudChannelToLabeledObjects(
      const sensor_msgs::PointCloud& point_cloud,
      std::vector<LabeledObject>& labeled_objects_vec);

  bool getSemanticLabelOfInstance(
      const LabeledObject &instance_labeled_object,
      const sensor_msgs::PointCloud &point_cloud,
      size_t &semantic_label);

  bool transLabeledObjectsToPointCloud2WithSemanticLabel(
      const LabeledObject &labeled_object,
      sensor_msgs::PointCloud2 &point_cloud2,
      const size_t &semantic_label);

  ros::NodeHandle nh_;

  std::vector<std::string> coco_names_vec_;

  std_msgs::Header current_header_;

  ros::Publisher objects_pub_;
};
 

#endif //POINT_CLOUD2_TO_OBJECT_VEC_CONVERTER_H
