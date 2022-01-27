#include "ViewPointExtractor.h"

#include <cstdlib>
#include <limits>
#include <memory>
#include <pcl/point_cloud.h>

bool ViewPointExtractor::getMultiView(
    std::vector<view_point_extractor::ViewPoint> &view_point_vec)
{
  view_point_vec.clear();

  // std::cout << "[INFO][ViewPointExtractor::getMultiView]\n" <<
    // "\t start call pointcloud2_to_object_vec_converter to get objects...\n";

  // objects[*].channels[*].name = [semantic_label]
  std::vector<sensor_msgs::PointCloud2> objects;
  if(!getObjects(objects))
  {
    std::cout << "[ERROR][ViewPointExtractor::getMultiView]\n" <<
      "\t getObjects failed!\n";

    return false;
  }
  // std::cout << "SUCCESS!\n";

  if(objects.size() == 0)
  {
    std::cout << "[WARN][ViewPointExtractor::getMultiView]\n" <<
      "\t no object found! please get view_ponit_vec later.\n";

    return true;
  }

  std::vector<sensor_msgs::PointCloud> trans_objects;
  if(!transPointCloud2VecToPointCloudVec(objects, trans_objects))
  {
    std::cout << "[ERROR][ViewPointExtractor::getMultiView]\n" <<
      "\t transPointCloud2VecToPointCloudVec failed!\n";

    return false;
  }

  std::vector<size_t> object_point_num_vec;
  object_point_num_vec.resize(objects.size());
  for(size_t i = 0; i < objects.size(); ++i)
  {
    object_point_num_vec[i] = trans_objects[i].points.size();
  }
  std::vector<size_t> object_point_num_more_to_less_vec;
  sortVecIndexByMoreToLess(object_point_num_vec, object_point_num_more_to_less_vec);

  // std::cout << "ViewPointExtractor::getMultiView :\n" <<
    // "Got " << objects.size() << " objects!\n";
  // for(size_t i = 0; i < object_point_num_more_to_less_vec.size(); ++i)
  // {
    // std::cout << "object[" << i << "] : size = "
      // << trans_objects[object_point_num_more_to_less_vec[i]].points.size()
      // << ", semantic = "
      // << coco_names_vec_[trans_objects[object_point_num_more_to_less_vec[i]].channels[0].values[0]]
      // << std::endl;
  // }

  // std::cout << "[INFO][ViewPointExtractor::getMultiView]\n" <<
    // "\t start get sampling point cloud vec...\n";

  std::vector<sensor_msgs::PointCloud> sampling_point_cloud_vec;
  getSamplingPointCloudVec(trans_objects, sampling_point_cloud_vec, grnet_input_pointcloud_size_);

  std::vector<sensor_msgs::PointCloud2> sampling_point_cloud2_vec;
  transPointCloudVecToPointCloud2Vec(sampling_point_cloud_vec, sampling_point_cloud2_vec);

  // std::cout << "SUCCESS!\n";

  // std::cout << "[INFO][ViewPointExtractor::getMultiView]\n" <<
    // "\t start call grnet_detect to get full point cloud2 vec...\n";

  std::vector<sensor_msgs::PointCloud2> full_point_cloud2_vec;
  if(!getFullPointCloud2VecFromPartialPointCloud2Vec(sampling_point_cloud2_vec, full_point_cloud2_vec))
  {
    std::cout << "[ERROR][ViewPointExtractor::getMultiView]\n" <<
      "\t getFullPointCloud2VecFromPartialPointCloud2Vec failed!\n";

    return false;
  }

  // std::cout << "SUCCESS!\n";

  std::vector<sensor_msgs::PointCloud> trans_full_point_cloud_vec;
  transPointCloud2VecToPointCloudVec(full_point_cloud2_vec, trans_full_point_cloud_vec);

  std::vector<view_point_extractor::ViewPoint> view_point_vec_to_show;

  const float view_point_diff_max = 2;
  const size_t view_point_similar_time = 1;
  const size_t object_disappear_count_max = 1;

  // std::cout << "[ERROR][ViewPointExtractor::getMultiView]\n" <<
    // "\t start search valid viewpoints...\n";

  object_saver_.resetObjectHistoryMatchState();

  for(size_t i = 0; i < trans_full_point_cloud_vec.size(); ++i)
  {
    view_point_extractor::ViewPoint next_best_view_point;
    getNextBestViewPoint(
        sampling_point_cloud_vec[i],
        trans_full_point_cloud_vec[i],
        next_best_view_point);

    float x_min = std::numeric_limits<float>::max();
    float y_min = std::numeric_limits<float>::max();
    float z_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::min();
    float y_max = std::numeric_limits<float>::min();
    float z_max = std::numeric_limits<float>::min();
    for(const geometry_msgs::Point32 &point : sampling_point_cloud_vec[i].points)
    {
      x_min = std::min(x_min, point.x);
      y_min = std::min(y_min, point.y);
      z_min = std::min(z_min, point.z);

      x_max = std::max(x_max, point.x);
      y_max = std::max(y_max, point.y);
      z_max = std::max(z_max, point.z);
    }

    BBox3D object_bbox;
    object_bbox.setPosition(x_min, y_min, z_min, x_max, y_max, z_max);
    ViewPoint3D object_view_point;
    object_view_point.setPosition(
        next_best_view_point.position.x,
        next_best_view_point.position.y,
        next_best_view_point.position.z,
        next_best_view_point.direction.x,
        next_best_view_point.direction.y,
        next_best_view_point.direction.z);

    if(!object_saver_.isObjectViewPointVaild(
          i,
          object_bbox,
          object_view_point,
          view_point_diff_max,
          view_point_similar_time))
    {
      next_best_view_point.is_visual = false;
    }
    else
    {
      view_point_vec.emplace_back(next_best_view_point);
    }

    view_point_vec_to_show.emplace_back(next_best_view_point);
  }

  object_saver_.updateObjectHistory(object_disappear_count_max);

  // std::cout << "SUCCESS!\n";

  if(false)
  {
    double arrow_length = 0.25;
    double text_scale = 0.05;
    double r, g, b;

    easy_pcl_visualzer_.createVis();
    easy_pcl_visualzer_.createViewPort(0, 0, 0.5, 1.0, "sampling point cloud");
    easy_pcl_visualzer_.createViewPort(0.5, 0.5, 1.0, 1.0, "grnet with all viewpoints");
    easy_pcl_visualzer_.createViewPort(0.5, 0, 1.0, 0.5, "grnet with selected viewpoints");
    for(size_t i = 0; i < sampling_point_cloud_vec.size(); ++i)
    {
      easy_pcl_visualzer_.createRandomColor(r, g, b);

      easy_pcl_visualzer_.addPointCloud(sampling_point_cloud_vec[i], "", 255, 255, 255, 0);
      easy_pcl_visualzer_.addPointCloud(trans_full_point_cloud_vec[i], "", r, g, b, 1);
      easy_pcl_visualzer_.addPointCloud(trans_full_point_cloud_vec[i], "", r, g, b, 2);

      pcl::PointXYZ start_point;
      pcl::PointXYZ end_point;
      start_point.x = view_point_vec_to_show[i].position.x - arrow_length * view_point_vec_to_show[i].direction.x;
      start_point.y = view_point_vec_to_show[i].position.y - arrow_length * view_point_vec_to_show[i].direction.y;
      start_point.z = view_point_vec_to_show[i].position.z - arrow_length * view_point_vec_to_show[i].direction.z;
      end_point.x = view_point_vec_to_show[i].position.x;
      end_point.y = view_point_vec_to_show[i].position.y;
      end_point.z = view_point_vec_to_show[i].position.z;

      easy_pcl_visualzer_.addText3D(
          coco_names_vec_[sampling_point_cloud_vec[i].channels[0].values[0]],
          start_point, text_scale, 255, 255, 255, "", 1);

      easy_pcl_visualzer_.addArrow(start_point, end_point, "", 0, 255, 0, 1);

      if(view_point_vec_to_show[i].is_visual)
      {
        easy_pcl_visualzer_.addText3D(
            coco_names_vec_[sampling_point_cloud_vec[i].channels[0].values[0]],
            start_point, text_scale, 255, 255, 255, "", 2);

        easy_pcl_visualzer_.addArrow(start_point, end_point, "", 0, 255, 0, 2);
      }
    }
    easy_pcl_visualzer_.showVis();
  }

  return true;
}

bool ViewPointExtractor::sortVecIndexByMoreToLess(
    const std::vector<size_t> &source_vec,
    std::vector<size_t> &sorted_index_vec)
{
  sorted_index_vec.clear();

  if(source_vec.size() == 0)
  {
    return true;
  }

  sorted_index_vec.emplace_back(0);

  for(size_t i = 1; i < source_vec.size(); ++i)
  {
    bool inserted_this_index = false;
    for(size_t j = 0; j < i; ++j)
    {
      if(source_vec[i] > source_vec[sorted_index_vec[j]])
      {
        sorted_index_vec.insert(sorted_index_vec.begin() + j, i);

        inserted_this_index = true;
        break;
      }
    }

    if(!inserted_this_index)
    {
      sorted_index_vec.emplace_back(i);
    }
  }

  return true;
}

bool ViewPointExtractor::getObjects(std::vector<sensor_msgs::PointCloud2> &objects)
{
  objects.clear();

  pointcloud2_to_object_vec_converter::PC2ToOBJS get_objects_from_pointcloud_serve;

  if(!pointcloud_to_objects_client_.call(get_objects_from_pointcloud_serve))
  {
    std::cout << "[ERROR][ViewPointExtractor::getObjects]\n" <<
      "\t get objects failed!\n";

    return false;
  }

  objects = get_objects_from_pointcloud_serve.response.objects;

  return true;
}

bool ViewPointExtractor::transPointCloud2VecToPointCloudVec(
    const std::vector<sensor_msgs::PointCloud2> &point_cloud2_vec,
    std::vector<sensor_msgs::PointCloud> &point_cloud_vec)
{
  point_cloud_vec.resize(point_cloud2_vec.size());
  for(size_t i = 0; i < point_cloud2_vec.size(); ++i)
  {
    if(!sensor_msgs::convertPointCloud2ToPointCloud(point_cloud2_vec[i], point_cloud_vec[i]))
    {
      std::cout << "[ERROR][ViewPointExtractor::transPointCloud2VecToPointCloudVec]\n" <<
        "\t trans point_cloud2[" << i << "] failed!\n";

      return false;
    }
  }

  return true;
}

bool ViewPointExtractor::transPointCloudVecToPointCloud2Vec(
    const std::vector<sensor_msgs::PointCloud> &point_cloud_vec,
    std::vector<sensor_msgs::PointCloud2> &point_cloud2_vec)
{
  point_cloud2_vec.resize(point_cloud_vec.size());
  for(size_t i = 0; i < point_cloud_vec.size(); ++i)
  {
    if(!sensor_msgs::convertPointCloudToPointCloud2(point_cloud_vec[i], point_cloud2_vec[i]))
    {
      std::cout << "[ERROR][ViewPointExtractor::transPointCloudVecToPointCloud2Vec]\n" <<
        "\t trans point_cloud[" << i << "] failed!\n";

      return false;
    }
  }

  return true;
}

bool ViewPointExtractor::getSamplingPointCloud(
    const sensor_msgs::PointCloud &source_point_cloud,
    sensor_msgs::PointCloud &sampling_point_cloud,
    const int &sampling_point_num)
{
  if(source_point_cloud.points.size() <= sampling_point_num)
  {
    sampling_point_cloud = source_point_cloud;
    return true;
  }

  if(sampling_point_num == 0)
  {
    sampling_point_cloud.points.clear();
    sampling_point_cloud.channels.clear();
    sampling_point_cloud.header = source_point_cloud.header;
    return true;
  }

  std::vector<size_t> remained_point_index_vec;
  remained_point_index_vec.emplace_back(0);

  std::vector<float> min_dist_to_remained_points_vec;
  min_dist_to_remained_points_vec.resize(
      source_point_cloud.points.size(),
      std::numeric_limits<float>::max());

  for(size_t i = 1; i < sampling_point_num; ++i)
  {
    const size_t &current_remained_point_index = remained_point_index_vec.back();
    const geometry_msgs::Point32 &new_added_point = source_point_cloud.points[current_remained_point_index];

    size_t new_remained_point_index;
    float max_dist_to_remained_points = 0;

    for(size_t j = 0; j < source_point_cloud.points.size(); ++j)
    {
      const geometry_msgs::Point32 &point = source_point_cloud.points[j];

      float current_dist_to_new_point =
        std::pow(point.x - new_added_point.x, 2) +
        std::pow(point.y - new_added_point.y, 2) +
        std::pow(point.z - new_added_point.z, 2);

      min_dist_to_remained_points_vec[j] =
        std::min(current_dist_to_new_point, min_dist_to_remained_points_vec[j]);

      if(min_dist_to_remained_points_vec[j] > max_dist_to_remained_points)
      {
        new_remained_point_index = j;
        max_dist_to_remained_points = min_dist_to_remained_points_vec[j];
      }
    }

    remained_point_index_vec.emplace_back(new_remained_point_index);
  }

  sampling_point_cloud.points.resize(sampling_point_num);
  sampling_point_cloud.channels.resize(source_point_cloud.channels.size());
  for(size_t i = 0; i < source_point_cloud.channels.size(); ++i)
  {
    sampling_point_cloud.channels[i].name = source_point_cloud.channels[i].name;
    sampling_point_cloud.channels[i].values.resize(sampling_point_num);
  }
  sampling_point_cloud.header = source_point_cloud.header;

  for(size_t i = 0; i < remained_point_index_vec.size(); ++i)
  {
    sampling_point_cloud.points[i] = source_point_cloud.points[remained_point_index_vec[i]];
    for(size_t j = 0; j < source_point_cloud.channels.size(); ++j)
    {
      sampling_point_cloud.channels[j].values[i] =
        source_point_cloud.channels[j].values[remained_point_index_vec[i]];
    }
  }

  return true;
}

bool ViewPointExtractor::getSamplingPointCloudVec(
    const std::vector<sensor_msgs::PointCloud> &source_point_cloud_vec,
    std::vector<sensor_msgs::PointCloud> &sampling_point_cloud_vec,
    const size_t &sampling_point_num)
{
  if(source_point_cloud_vec.size() == 0)
  {
    sampling_point_cloud_vec.clear();
    return true;
  }

  // old cpu method
  // sampling_point_cloud_vec.resize(source_point_cloud_vec.size());
  // for(size_t i = 0; i < source_point_cloud_vec.size(); ++i)
  // {
  //   getSamplingPointCloud(source_point_cloud_vec[i], sampling_point_cloud_vec[i], sampling_point_num);
  // }

  std::vector<sensor_msgs::PointCloud2> point_cloud2_vec;
  transPointCloudVecToPointCloud2Vec(source_point_cloud_vec, point_cloud2_vec);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> source_pcl_point_cloud_vec;
  source_pcl_point_cloud_vec.resize(source_point_cloud_vec.size());
  for(size_t i = 0; i < source_point_cloud_vec.size(); ++i)
  {
    pcl::fromROSMsg(point_cloud2_vec[i], source_pcl_point_cloud_vec[i]);
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>> sampling_pcl_point_cloud_vec;

  farthest_sampling::samplePointCloudsCuda(source_pcl_point_cloud_vec, sampling_pcl_point_cloud_vec, sampling_point_num);

  std::vector<sensor_msgs::PointCloud2> sampling_point_cloud2_vec;
  sampling_point_cloud2_vec.resize(sampling_pcl_point_cloud_vec.size());
  for(size_t i = 0; i < sampling_pcl_point_cloud_vec.size(); ++i)
  {
    pcl::toROSMsg(sampling_pcl_point_cloud_vec[i], sampling_point_cloud2_vec[i]);
  }
  transPointCloud2VecToPointCloudVec(sampling_point_cloud2_vec, sampling_point_cloud_vec);

  for(size_t pc_idx = 0; pc_idx < source_point_cloud_vec.size(); ++pc_idx)
  {
    const sensor_msgs::PointCloud &source_point_cloud = source_point_cloud_vec[pc_idx];
    sensor_msgs::PointCloud &sampling_point_cloud = sampling_point_cloud_vec[pc_idx];

    sampling_point_cloud.channels.resize(source_point_cloud.channels.size());
    for(size_t i = 0; i < source_point_cloud.channels.size(); ++i)
    {
      sampling_point_cloud.channels[i].name = source_point_cloud.channels[i].name;
      sampling_point_cloud.channels[i].values.resize(sampling_point_num);
    }
    sampling_point_cloud.header = source_point_cloud.header;

    for(size_t i = 0; i < sampling_point_cloud.points.size(); ++i)
    {
      for(size_t j = 0; j < sampling_point_cloud.channels.size(); ++j)
      {
        sampling_point_cloud.channels[j].values[i] =
          source_point_cloud.channels[j].values[0];
      }
    }
  }

  return true;
}

bool ViewPointExtractor::getFullPointCloud2FromPartialPointCloud2(
    const sensor_msgs::PointCloud2 &partial_point_cloud2,
    sensor_msgs::PointCloud2 &full_point_cloud2)
{
  grnet_detect::PC2ToPC2 get_full_pointcloud_serve;
  get_full_pointcloud_serve.request.partial_cloud = partial_point_cloud2;

  if(!grnet_detector_client_.call(get_full_pointcloud_serve))
  {
    std::cout << "[ERROR][ViewPointExtractor::getFullPointCloud2FromPartialPointCloud2]\n" <<
      "\t call grnet_detector failed!\n";

    return false;
  }

  full_point_cloud2 = get_full_pointcloud_serve.response.complete_cloud;

  return true;
}

bool ViewPointExtractor::getFullPointCloud2VecFromPartialPointCloud2Vec(
    const std::vector<sensor_msgs::PointCloud2> &partial_point_cloud2_vec,
    std::vector<sensor_msgs::PointCloud2> &full_point_cloud2_vec)
{
  full_point_cloud2_vec.resize(partial_point_cloud2_vec.size());

  for(size_t i = 0; i < partial_point_cloud2_vec.size(); ++i)
  {
    if(!getFullPointCloud2FromPartialPointCloud2(partial_point_cloud2_vec[i], full_point_cloud2_vec[i]))
    {
      std::cout << "[ERROR][ViewPointExtractor::getFullPointCloud2VecFromPartialPointCloud2Vec]\n" <<
        "\t get full pointcloud2 from partial pointcloud2 failed!\n";

      return false;
    }
  }

  return true;
}

bool ViewPointExtractor::getMaxDistFullPointPosition(
    const sensor_msgs::PointCloud &partial_point_cloud,
    const sensor_msgs::PointCloud &full_point_cloud,
    view_point_extractor::ViewPoint &next_best_view_point)
{
  if(partial_point_cloud.points.size() == 0)
  {
    std::cout << "[ERROR][ViewPointExtractor::getMaxDistSumFullPoint]\n" <<
      "\t input partial_point_cloud is empty!\n";
    return false;
  }
  if(full_point_cloud.points.size() == 0)
  {
    std::cout << "[ERROR][ViewPointExtractor::getMaxDistSumFullPoint]\n" <<
      "\t input full_point_cloud is empty!" << std::endl;
    return false;
  }

  float min_dist_to_partial_point_cloud_sum = 0;

  size_t max_dist_full_point_idx;
  float max_dist_to_partial_point_cloud = 0;

  for(size_t i = 0; i < full_point_cloud.points.size(); ++i)
  {
    const geometry_msgs::Point32 &full_point = full_point_cloud.points[i];

    float current_min_dist_sum_to_partial_point_cloud = std::numeric_limits<float>::max();
    for(const geometry_msgs::Point32 &partial_point : partial_point_cloud.points)
    {
      float current_dist_to_partial_point = sqrt(
          std::pow(full_point.x - partial_point.x, 2) +
          std::pow(full_point.y - partial_point.y, 2) +
          std::pow(full_point.z - partial_point.z, 2));

      current_min_dist_sum_to_partial_point_cloud =
        std::fmin(current_min_dist_sum_to_partial_point_cloud, current_dist_to_partial_point);
    }

    if(current_min_dist_sum_to_partial_point_cloud > max_dist_to_partial_point_cloud)
    {
      max_dist_to_partial_point_cloud = current_min_dist_sum_to_partial_point_cloud;
      max_dist_full_point_idx = i;
    }

    min_dist_to_partial_point_cloud_sum += current_min_dist_sum_to_partial_point_cloud;
  }

  next_best_view_point.position = full_point_cloud.points[max_dist_full_point_idx];

  float average_min_dist_to_partial_point_cloud =
    min_dist_to_partial_point_cloud_sum / full_point_cloud.points.size();

  if(max_dist_to_partial_point_cloud / average_min_dist_to_partial_point_cloud > 10)
  {
    next_best_view_point.is_visual = true;
  }

  return true;
}

bool ViewPointExtractor::getNextBestViewPointPosition(
    const sensor_msgs::PointCloud &partial_point_cloud,
    const sensor_msgs::PointCloud &full_point_cloud,
    view_point_extractor::ViewPoint &next_best_view_point)
{
  getMaxDistFullPointPosition(partial_point_cloud, full_point_cloud, next_best_view_point);

  return true;
}

bool ViewPointExtractor::getNextBestViewPointDirection(
    const sensor_msgs::PointCloud &point_cloud,
    view_point_extractor::ViewPoint &next_best_view_point)
{
  if(point_cloud.points.size() == 0)
  {
    std::cout << "[ERROR][ViewPointExtractor::getMaxDistSumFullPoint]\n" <<
      "\t input partial_point_cloud is empty!" << std::endl;
    return false;
  }

  geometry_msgs::Point32 center_point;
  center_point.x = 0;
  center_point.y = 0;
  center_point.z = 0;

  for(const geometry_msgs::Point32 &point : point_cloud.points)
  {
    center_point.x += point.x;
    center_point.y += point.y;
    center_point.z += point.z;
  }
  center_point.x /= point_cloud.points.size();
  center_point.y /= point_cloud.points.size();
  center_point.z /= point_cloud.points.size();

  next_best_view_point.direction.x = center_point.x - next_best_view_point.position.x;
  next_best_view_point.direction.y = center_point.y - next_best_view_point.position.y;
  next_best_view_point.direction.z = center_point.z - next_best_view_point.position.z;

  const float view_point_direction_length = sqrt(
      std::pow(next_best_view_point.direction.x, 2) +
      std::pow(next_best_view_point.direction.y, 2) +
      std::pow(next_best_view_point.direction.z, 2));
  next_best_view_point.direction.x /= view_point_direction_length;
  next_best_view_point.direction.y /= view_point_direction_length;
  next_best_view_point.direction.z /= view_point_direction_length;

  return true;
}

bool ViewPointExtractor::getNextBestViewPoint(
    const sensor_msgs::PointCloud &partial_point_cloud,
    const sensor_msgs::PointCloud &full_point_cloud,
    view_point_extractor::ViewPoint &next_best_view_point)
{
  next_best_view_point.is_visual = false;

  if(partial_point_cloud.points.size() == 0)
  {
    std::cout << "[ERROR][ViewPointExtractor::getNextBestViewPoint]\n" <<
      "\t input partial_point_cloud is empty!" << std::endl;
    return false;
  }
  if(full_point_cloud.points.size() == 0)
  {
    std::cout << "[ERROR][ViewPointExtractor::getNextBestViewPoint]\n" <<
      "\t input full_point_cloud is empty!" << std::endl;
    return false;
  }

  getNextBestViewPointPosition(
      partial_point_cloud,
      full_point_cloud,
      next_best_view_point);

  getNextBestViewPointDirection(
      partial_point_cloud,
      next_best_view_point);

  return true;
}

