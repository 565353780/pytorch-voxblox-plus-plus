#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace farthest_sampling
{
void samplePointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_point_cloud,
                      pcl::PointCloud<pcl::PointXYZ>& sampled_point_cloud, unsigned sampling_point_num);

void samplePointCloudsCuda(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& input_point_clouds,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>>& sampled_point_clouds,
                           unsigned sampling_point_num, bool need_print_device_properties = false);

void farthestPointSamplingLauncher(unsigned batch_size, unsigned sampling_point_num, const unsigned* point_nums,
                                   const float* input_point_clouds, float* distances, unsigned* output_indices);
} // namespace farthest_sampling
