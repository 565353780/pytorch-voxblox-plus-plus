#include <iostream>
#include <limits>
#include <pcl/point_cloud.h>
#include <cuda_runtime_api.h>
#include "farthest_sampling/farthest_sampling.h"

namespace farthest_sampling
{
pcl::PointCloud<pcl::PointXYZ> constructMergedInputPointCloud(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& input_point_clouds,
    const std::vector<std::size_t>& need_sampling_point_cloud_indices, unsigned* point_cloud_sizes);

void fillSampledPointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& input_point_clouds,
                            std::vector<pcl::PointCloud<pcl::PointXYZ>>& sampled_point_clouds,
                            unsigned sampling_point_num,
                            const std::vector<std::size_t>& need_sampling_point_cloud_indices,
                            unsigned* output_indices);

void printDeviceProperties(const cudaDeviceProp& prop);
int initializeCudaDevice(bool need_print_device_properties);

void samplePointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_point_cloud,
                      pcl::PointCloud<pcl::PointXYZ>& sampled_point_cloud, unsigned sampling_point_num)
{
  if (input_point_cloud.points.size() <= sampling_point_num)
  {
    sampled_point_cloud = input_point_cloud;
    return;
  }

  if (sampling_point_num == 0)
  {
    std::cout << "Sampling point num is 0, do nothing in sampling process." << std::endl;
    sampled_point_cloud.points.clear();
    return;
  }

  std::vector<unsigned> sampled_point_indices;
  sampled_point_indices.push_back(0);

  std::vector<float> min_dist_to_sampled_points(input_point_cloud.size(), std::numeric_limits<float>::max());

  for (unsigned i = 1; i < sampling_point_num; ++i)
  {
    auto latest_added_point = input_point_cloud.points[sampled_point_indices.back()];
    int candidate_point_index = -1;
    float max_dist_to_sampled_points = 0;
    for (int j = 0; j < input_point_cloud.points.size(); ++j)
    {
      auto point = input_point_cloud.points[j];
      float current_dist_to_latest_point = std::pow(point.x - latest_added_point.x, 2) +
                                           std::pow(point.y - latest_added_point.y, 2) +
                                           std::pow(point.z - latest_added_point.z, 2);

      min_dist_to_sampled_points[j] = std::min(current_dist_to_latest_point, min_dist_to_sampled_points[j]);
      if (min_dist_to_sampled_points[j] > max_dist_to_sampled_points)
      {
        candidate_point_index = j;
        max_dist_to_sampled_points = min_dist_to_sampled_points[j];
      }
    }
    sampled_point_indices.emplace_back(candidate_point_index);
  }

  sampled_point_cloud.points.resize(sampling_point_num);
  sampled_point_cloud.header = input_point_cloud.header;

  for (std::size_t i = 0; i < sampled_point_indices.size(); ++i)
  {
    sampled_point_cloud.points[i] = input_point_cloud.points[sampled_point_indices[i]];
  }
}

void samplePointCloudsCuda(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& input_point_clouds,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>>& sampled_point_clouds,
                           unsigned sampling_point_num, bool need_print_device_properties)
{
  if (input_point_clouds.size() == 0)
  {
    return;
  }

  sampled_point_clouds.resize(input_point_clouds.size(), pcl::PointCloud<pcl::PointXYZ>());

  std::vector<std::size_t> need_sampling_point_cloud_indices;
  for (std::size_t i = 0; i < input_point_clouds.size(); ++i)
  {
    if (input_point_clouds[i].points.size() >= sampling_point_num)
    {
      need_sampling_point_cloud_indices.push_back(i);
    }
    else
    {
      sampled_point_clouds[i] = input_point_clouds[i];
    }
  }

  std::size_t need_sampling_point_cloud_num = need_sampling_point_cloud_indices.size();
  if (!need_sampling_point_cloud_num)
  {
    return;
  }

  auto point_cloud_sizes = std::make_unique<unsigned[]>(need_sampling_point_cloud_num);

  auto merged_input_point_clouds =
      constructMergedInputPointCloud(input_point_clouds, need_sampling_point_cloud_indices, point_cloud_sizes.get());

  auto device_id = initializeCudaDevice(need_print_device_properties);

  std::size_t input_point_count = merged_input_point_clouds.size();
  float* input_point_cloud;
  unsigned* input_point_nums;
  float* distances;
  unsigned* output_indices;

  cudaMallocManaged((void**)&input_point_cloud, 4 * input_point_count * sizeof(float));
  cudaMallocManaged((void**)&input_point_nums, need_sampling_point_cloud_num * sizeof(unsigned));
  cudaMallocManaged((void**)&distances, input_point_count * sizeof(float));
  cudaMallocManaged((void**)&output_indices, need_sampling_point_cloud_num * sampling_point_num * sizeof(unsigned));

  cudaMemcpy(input_point_cloud, merged_input_point_clouds.points.data(), 4 * input_point_count * sizeof(float),
             cudaMemcpyHostToDevice);
  cudaMemcpy(input_point_nums, point_cloud_sizes.get(), need_sampling_point_cloud_num * sizeof(unsigned),
             cudaMemcpyHostToDevice);

  cudaMemPrefetchAsync(input_point_cloud, 4 * input_point_count * sizeof(float), device_id);
  cudaMemPrefetchAsync(input_point_nums, need_sampling_point_cloud_num * sizeof(unsigned), device_id);
  cudaMemPrefetchAsync(distances, input_point_count * sizeof(float), device_id);
  cudaMemPrefetchAsync(output_indices, need_sampling_point_cloud_num * sampling_point_num * sizeof(unsigned),
                       device_id);

  // std::cout << "need_sampling_point_cloud_num = " << need_sampling_point_cloud_num << std::endl;

  farthestPointSamplingLauncher(need_sampling_point_cloud_num, sampling_point_num, input_point_nums, input_point_cloud,
                                distances, output_indices);

  cudaDeviceSynchronize();

  fillSampledPointClouds(input_point_clouds, sampled_point_clouds, sampling_point_num,
                         need_sampling_point_cloud_indices, output_indices);

  // if (need_sampling_point_cloud_num > 1)
  // {
    // std::cout << "samplePointCloudsCuda : output indices check : " << std::endl;
    // for (size_t i = 0; i < need_sampling_point_cloud_num; ++i)
    // {
      // std::cout << output_indices[i * sampling_point_num + 1] << " , ";
    // }
    // std::cout << std::endl;
  // }

  cudaFree(input_point_cloud);
  cudaFree(input_point_nums);
  cudaFree(distances);
  cudaFree(output_indices);
}

pcl::PointCloud<pcl::PointXYZ> constructMergedInputPointCloud(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& input_point_clouds,
    const std::vector<std::size_t>& need_sampling_point_cloud_indices, unsigned* point_cloud_sizes)
{
  pcl::PointCloud<pcl::PointXYZ> result;

  for (std::size_t i = 0; i < need_sampling_point_cloud_indices.size(); ++i)
  {
    const pcl::PointCloud<pcl::PointXYZ>& need_sampling_point_cloud =
        input_point_clouds[need_sampling_point_cloud_indices[i]];

    point_cloud_sizes[i] = need_sampling_point_cloud.points.size();

    result.points.insert(result.end(), need_sampling_point_cloud.points.begin(), need_sampling_point_cloud.end());
  }
  return result;
}

void fillSampledPointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& input_point_clouds,
                            std::vector<pcl::PointCloud<pcl::PointXYZ>>& sampled_point_clouds,
                            unsigned sampling_point_num,
                            const std::vector<std::size_t>& need_sampling_point_cloud_indices, unsigned* output_indices)
{
  for (std::size_t i = 0; i < need_sampling_point_cloud_indices.size(); ++i)
  {
    auto need_sampling_point_cloud_index = need_sampling_point_cloud_indices[i];
    const pcl::PointCloud<pcl::PointXYZ>& need_sampling_source_point_cloud =
        input_point_clouds[need_sampling_point_cloud_index];

    for (size_t j = 0; j < sampling_point_num; ++j)
    {
      const size_t sampled_point_index = output_indices[i * sampling_point_num + j];

      sampled_point_clouds[need_sampling_point_cloud_index].points.push_back(
          need_sampling_source_point_cloud.points[sampled_point_index]);
    }
  }
}

int initializeCudaDevice(bool need_print_device_properties)
{
  int count, device_id;
  // get the cuda device count
  cudaGetDeviceCount(&count);
  for (device_id = 0; device_id < count; ++device_id)
  {
    cudaDeviceProp prop;
    if (cudaGetDeviceProperties(&prop, device_id) == cudaSuccess)
    {
      if (prop.major >= 1)
      {
        if (need_print_device_properties)
        {
          printDeviceProperties(prop);
        }
        break;
      }
    }
  }
  cudaSetDevice(device_id);
  return device_id;
}

void printDeviceProperties(const cudaDeviceProp& prop)
{
  std::cout << "Device Name : " << prop.name << ".\n";
  std::cout << "totalGlobalMem : " << prop.totalGlobalMem << ".\n";
  std::cout << "sharedMemPerBlock : " << prop.sharedMemPerBlock << ".\n";
  std::cout << "regsPerBlock : " << prop.regsPerBlock << ".\n";
  std::cout << "warpSize : " << prop.warpSize << ".\n";
  std::cout << "memPitch : " << prop.memPitch << ".\n";
  std::cout << "maxThreadsPerBlock : " << prop.maxThreadsPerBlock << ".\n";
  std::cout << "maxThreadsDim[0 - 2] : " << prop.maxThreadsDim[0] << " " << prop.maxThreadsDim[1] << " "
            << prop.maxThreadsDim[2] << ".\n";
  std::cout << "maxGridSize[0 - 2] : " << prop.maxGridSize[0] << " " << prop.maxGridSize[1] << " "
            << prop.maxGridSize[2] << ".\n";
  std::cout << "totalConstMem : " << prop.totalConstMem << ".\n";
  std::cout << "major.minor : " << prop.major << "." << prop.minor << ".\n";
  std::cout << "clockRate : " << prop.clockRate << ".\n";
  std::cout << "textureAlignment : " << prop.textureAlignment << ".\n";
  std::cout << "deviceOverlap : " << prop.deviceOverlap << ".\n";
  std::cout << "multiProcessorCount : " << prop.multiProcessorCount << '.' << std::endl;
}
} // namespace farthest_sampling
