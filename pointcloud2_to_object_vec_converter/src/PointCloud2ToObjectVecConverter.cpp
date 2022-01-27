#include "PointCloud2ToObjectVecConverter.h"

bool LabeledObject::reset()
{
  instance_label = 0;

  points.clear();

  return true;
}

bool LabeledObject::setLabel(
    const size_t &new_instance_label)
{
  instance_label = new_instance_label;

  return true;
}

bool LabeledObject::addPoint(
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
    const std::uint8_t &semantic_label)
{
  PointWithRGBAndLabel new_point;
  new_point.x = point_x;
  new_point.y = point_y;
  new_point.z = point_z;
  new_point.r = point_r;
  new_point.g = point_g;
  new_point.b = point_b;
  new_point.a = point_a,
  new_point.distance = point_distance;
  new_point.weight = point_weight;
  new_point.segment_label = segment_label;
  new_point.semantic_label = semantic_label;
  new_point.instance_label = instance_label;

  points.emplace_back(new_point);

  return true;
}

bool LabeledObject::getBBox(
    BBox &bbox)
{
  if(points.size() == 0)
  {
    std::cout << "LabeledObject::getBBox : " <<
      "point_num is 0!" << std::endl;

    return false;
  }

  bbox.x_min = std::numeric_limits<float>::max();
  bbox.x_max = std::numeric_limits<float>::min();
  bbox.y_min = std::numeric_limits<float>::max();
  bbox.y_max = std::numeric_limits<float>::min();

  bbox.is_valid = true;

  for(const PointWithRGBAndLabel& point : points)
  {
    bbox.x_min = std::fmin(bbox.x_min, point.x);
    bbox.x_max = std::fmax(bbox.x_max, point.x);
    bbox.y_min = std::fmin(bbox.y_min, point.y);
    bbox.y_max = std::fmax(bbox.y_max, point.y);
  }

  return true;
}

bool PointCloud2ToObjectVecConverter::splitPointCloudChannelToLabeledObjects(
    const sensor_msgs::PointCloud &point_cloud,
    std::vector<LabeledObject> &labeled_objects_vec)
{
  if(point_cloud.points.size() == 0)
  {
    std::cout << "PointCloud2ToObjectVecConverter::splitPointCloudChannelToLabeledObjects : " <<
      "point_cloud is empty!" << std::endl;

    return false;
  }

  // point_cloud.channels[*].name = [r, g, b, a, distance, weight, segment_label, semantic_class, instance_label]
  for(size_t i = 0; i < point_cloud.points.size(); ++i)
  {
    const float &current_x = point_cloud.points[i].x;
    const float &current_y = point_cloud.points[i].y;
    const float &current_z = point_cloud.points[i].z;
    const std::uint8_t &current_r = point_cloud.channels[0].values[i];
    const std::uint8_t &current_g = point_cloud.channels[1].values[i];
    const std::uint8_t &current_b = point_cloud.channels[2].values[i];
    const std::uint8_t &current_a = point_cloud.channels[3].values[i];
    const float &current_distance = point_cloud.channels[4].values[i];
    const float &current_weight = point_cloud.channels[5].values[i];
    const std::uint16_t &current_segment_label = point_cloud.channels[6].values[i];
    const std::uint8_t &current_semantic_label = point_cloud.channels[7].values[i];
    const std::uint16_t &current_instance_label = point_cloud.channels[8].values[i];

    bool find_this_label = false;
    for(size_t j = 0; j < labeled_objects_vec.size(); ++j)
    {
      if(current_instance_label == labeled_objects_vec[j].instance_label)
      {
        labeled_objects_vec[j].addPoint(
            current_x, current_y, current_z,
            current_r, current_g, current_b, current_a,
            current_distance, current_weight,
            current_segment_label, current_semantic_label);
        find_this_label = true;
        break;
      }
    }

    if(!find_this_label)
    {
      LabeledObject new_labeled_objects;
      new_labeled_objects.setLabel(current_instance_label);
      new_labeled_objects.addPoint(
          current_x, current_y, current_z,
          current_r, current_g, current_b, current_a,
          current_distance, current_weight,
          current_segment_label, current_semantic_label);
      labeled_objects_vec.emplace_back(new_labeled_objects);
    }
  }

  return true;
}

bool PointCloud2ToObjectVecConverter::getSemanticLabelOfInstance(
    const LabeledObject &instance_labeled_object,
    const sensor_msgs::PointCloud &point_cloud,
    size_t &semantic_label)
{
  if(instance_labeled_object.points.size() == 0)
  {
    std::cout << "PointCloud2ToObjectVecConverter::getSemanticLabelOfInstance : " <<
      "instance_labeled_object is empty!" << std::endl;

    return false;
  }

  if(point_cloud.points.size() == 0)
  {
    std::cout << "PointCloud2ToObjectVecConverter::getSemanticLabelOfInstance : " <<
      "point_cloud is empty!" << std::endl;

    return false;
  }

  // data type : [[semantic_label_1, count], [semantic_label_2, count], ...]
  std::vector<std::pair<size_t, size_t>> semantic_label_count_vec;

  for(const PointWithRGBAndLabel& point : instance_labeled_object.points)
  {
    const size_t &current_semantic_label = point.semantic_label;

    bool find_this_label = false;

    for(std::pair<size_t, size_t> &semantic_label_count : semantic_label_count_vec)
    {
      if(current_semantic_label == semantic_label_count.first)
      {
        ++semantic_label_count.second;
        find_this_label = true;
        break;
      }
    }

    if(!find_this_label)
    {
      std::pair<size_t, size_t> new_semantic_label_count;
      new_semantic_label_count.first = current_semantic_label;
      new_semantic_label_count.second = 1;
      semantic_label_count_vec.emplace_back(new_semantic_label_count);
    }
  }

  // if(semantic_label_count_vec.size() != 1)
  // {
    // std::cout << "============================" << std::endl;
    // std::cout << "getSemanticLabelOfInstance -> semantic_label_count_vec :" << std::endl
      // << "Size = " << semantic_label_count_vec.size() << std::endl;
    // for(std::pair<size_t, size_t> &semantic_label_count : semantic_label_count_vec)
    // {
      // std::cout << "semantic_label_count_vec.label = " << coco_names_vec_[semantic_label_count.first]
        // << ", size = " << semantic_label_count.second << std::endl;
    // }
    // std::cout << "============================" << std::endl;
  // }

  size_t max_size_semantic_index;
  size_t max_semantic_num = 0;

  for(size_t i = 0; i < semantic_label_count_vec.size(); ++i)
  {
    if(semantic_label_count_vec[i].second > max_semantic_num)
    {
      max_size_semantic_index = i;
      max_semantic_num = semantic_label_count_vec[i].second;
    }
  }

  semantic_label = semantic_label_count_vec[max_size_semantic_index].first;

  return true;
}

bool PointCloud2ToObjectVecConverter::transLabeledObjectsToPointCloud2WithSemanticLabel(
    const LabeledObject &labeled_object,
    sensor_msgs::PointCloud2 &point_cloud2,
    const size_t &semantic_label)
{
  if(labeled_object.points.size() == 0)
  {
    std::cout << "PointCloud2ToObjectVecConverter::transLabeledObjectsToPointCloud2WithSemanticLabel : " <<
      "labeled_object is empty!" << std::endl;

    return false;
  }

  pcl::PointCloud<PointWithRGBAndLabel> pcl_point_cloud;

  pcl_point_cloud.width = labeled_object.points.size();
  pcl_point_cloud.height = 1;

  pcl_point_cloud.points.resize(pcl_point_cloud.width * pcl_point_cloud.height);

  for(size_t i = 0; i < pcl_point_cloud.points.size(); ++i)
  {
    pcl_point_cloud.points[i] = labeled_object.points[i];
  }

  pcl::toROSMsg(pcl_point_cloud, point_cloud2);

  point_cloud2.header = current_header_;

  return true;
}

bool PointCloud2ToObjectVecConverter::transPointCloud2ToObjects(
    const sensor_msgs::PointCloud2 &point_cloud2,
    std::vector<sensor_msgs::PointCloud2> &objects)
{
  if(point_cloud2.data.size() == 0)
  {
    std::cout << "PointCloud2ToObjectVecConverter::transPointCloud2ToObjects : " <<
      "point_cloud2 is empty!" << std::endl;

    return false;
  }

  current_header_ = point_cloud2.header;

  // point_cloud.channels[*].name = [r, g, b, a, distance, weight, segment_label, semantic_class, instance_label]
  sensor_msgs::PointCloud point_cloud;
  convertPointCloud2ToPointCloud(point_cloud2, point_cloud);

  std::vector<LabeledObject> instance_objects_vec;
  splitPointCloudChannelToLabeledObjects(point_cloud, instance_objects_vec);

  for(const LabeledObject &instance_object : instance_objects_vec)
  {
    sensor_msgs::PointCloud2 current_instance_point_cloud2;

    size_t current_instance_semantic_label;

    getSemanticLabelOfInstance(instance_object, point_cloud, current_instance_semantic_label);

    if(current_instance_semantic_label != 0)
    {
      transLabeledObjectsToPointCloud2WithSemanticLabel(
          instance_object,
          current_instance_point_cloud2,
          current_instance_semantic_label);

      objects.emplace_back(current_instance_point_cloud2);
    }
  }

  return true;
}

