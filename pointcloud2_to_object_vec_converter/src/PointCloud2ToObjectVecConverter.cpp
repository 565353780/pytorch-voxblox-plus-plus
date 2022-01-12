#include "PointCloud2ToObjectVecConverter.h"

bool LabeledObject::reset()
{
  point_num = 0;
  label = 0;

  index.clear();
  x.clear();
  y.clear();
  z.clear();

  return true;
}

bool LabeledObject::setLabel(
    const size_t &new_label)
{
  label = new_label;

  return true;
}

bool LabeledObject::addPoint(
    const size_t &point_index,
    const float &point_x,
    const float &point_y,
    const float &point_z)
{
  index.emplace_back(point_index);
  x.emplace_back(point_x);
  y.emplace_back(point_y);
  z.emplace_back(point_z);
  ++point_num;

  return true;
}

bool LabeledObject::isIndexInThis(
    const size_t &point_index)
{
  std::vector<size_t>::const_iterator find_itr = find(index.cbegin(), index.cend(), point_index);

  if(find_itr == index.cend())
  {
    return false;
  }

  return true;
}

bool LabeledObject::getBBox(
    BBox &bbox)
{
  if(point_num == 0)
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

  for(size_t i = 0; i < point_num; ++i)
  {
    bbox.x_min = std::fmin(bbox.x_min, x[i]);
    bbox.x_max = std::fmax(bbox.x_max, x[i]);
    bbox.y_min = std::fmin(bbox.y_min, y[i]);
    bbox.y_max = std::fmax(bbox.y_max, y[i]);
  }

  return true;
}

float LabeledObject::getDist(
    const float &x1,
    const float &y1,
    const float &z1,
    const float &x2,
    const float &y2,
    const float &z2)
{
  float dist = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

  return dist;
}

float LabeledObject::getDistByIndex(
    const size_t &point_index,
    const float &point_x,
    const float &point_y,
    const float &point_z)
{
  if(point_index >= point_num)
  {
    return -1;
  }

  const float dist = getDist(x[point_index], y[point_index], z[point_index], point_x, point_y, point_z);

  return dist;
}

float LabeledObject::getMinDist(
    const float &point_x,
    const float &point_y,
    const float &point_z)
{
  float min_dist = std::numeric_limits<float>::max();

  if(point_num == 0)
  {
    std::cout << "LabeledObject::getMinDist : " <<
      "current point_num is 0!" << std::endl;
    return 0;
  }

  for(size_t i = 0; i < point_num; ++i)
  {
    const float current_dist = getDistByIndex(i, point_x, point_y, point_z);

    min_dist = std::fmin(min_dist, current_dist);
  }

  return min_dist;
}

bool PointCloud2ToObjectVecConverter::isSamePoint(
    const geometry_msgs::Point32 &point1,
    const geometry_msgs::Point32 &point2)
{
  if(point1.x == point2.x && point1.y == point2.y && point1.z == point2.z)
  {
    return true;
  }

  return false;
}

bool PointCloud2ToObjectVecConverter::splitPointCloudChannelToLabeledObjects(
    const sensor_msgs::PointCloud &point_cloud,
    const size_t &channel_index,
    std::vector<LabeledObject> &labeled_objects_vec)
{
  if(point_cloud.points.size() == 0)
  {
    std::cout << "PointCloud2ToObjectVecConverter::splitPointCloudChannelToLabeledObjects : " <<
      "point_cloud is empty!" << std::endl;

    return false;
  }

  for(size_t i = 0; i < point_cloud.points.size(); ++i)
  {
    const float &current_x = point_cloud.points[i].x;
    const float &current_y = point_cloud.points[i].y;
    const float &current_z = point_cloud.points[i].z;
    const size_t &current_label = point_cloud.channels[channel_index].values[i];

    bool find_this_label = false;
    for(size_t j = 0; j < labeled_objects_vec.size(); ++j)
    {
      if(current_label == labeled_objects_vec[j].label)
      {
        labeled_objects_vec[j].addPoint(i, current_x, current_y, current_z);
        find_this_label = true;
        break;
      }
    }

    if(!find_this_label)
    {
      LabeledObject new_labeled_objects;
      new_labeled_objects.setLabel(current_label);
      new_labeled_objects.addPoint(i, current_x, current_y, current_z);
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
  if(instance_labeled_object.point_num == 0)
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

  for(const size_t &point_index : instance_labeled_object.index)
  {
    const size_t &current_semantic_label = point_cloud.channels[3].values[point_index];

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
  if(labeled_object.point_num == 0)
  {
    std::cout << "PointCloud2ToObjectVecConverter::transLabeledObjectsToPointCloud2WithSemanticLabel : " <<
      "labeled_object is empty!" << std::endl;

    return false;
  }

  pcl::PointCloud<PointCloudWithSemanticAndInstanceLabel> pcl_point_cloud;

  pcl_point_cloud.width = labeled_object.point_num;
  pcl_point_cloud.height = 1;

  pcl_point_cloud.points.resize(pcl_point_cloud.width * pcl_point_cloud.height);

  for(size_t i = 0; i < pcl_point_cloud.points.size(); ++i)
  {
    pcl_point_cloud.points[i].x = labeled_object.x[i];
    pcl_point_cloud.points[i].y = labeled_object.y[i];
    pcl_point_cloud.points[i].z = labeled_object.z[i];
    pcl_point_cloud.points[i].semantic_label = semantic_label;
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

  // point_cloud.channels[*].name = [distance, weight, segment_label, semantic_class, instance_label]
  sensor_msgs::PointCloud point_cloud;
  convertPointCloud2ToPointCloud(point_cloud2, point_cloud);

  std::cout << "point_cloud size = " << point_cloud.points.size() << std::endl;

  std::vector<LabeledObject> instance_objects_vec;
  splitPointCloudChannelToLabeledObjects(point_cloud, 4, instance_objects_vec);

  std::cout << "instance_objects_vec size = " << instance_objects_vec.size() << std::endl;

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


