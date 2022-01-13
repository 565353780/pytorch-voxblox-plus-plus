#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include "farthest_sampling/farthest_sampling.h"

#define STRX(x) #x
#define STR(x) STRX(x)

TEST(SamplingTest, farthestSamplingWithSinglePointCloud)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>> input_point_clouds(1);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> output_point_clouds;

  ASSERT_NE(pcl::io::loadPLYFile<pcl::PointXYZ>(STR(INPUT_POINT_CLOUD_PATH),
            input_point_clouds[0]), -1) << "Couldn't read file test point cloud file";
  farthest_sampling::samplePointCloudsCuda(input_point_clouds, output_point_clouds, 4096);
  boost::filesystem::path output_path = STR(OUTPUT_POINT_CLOUD_PATH);
  if (output_path.has_parent_path() && !boost::filesystem::exists(output_path.parent_path()))
  {
    boost::filesystem::create_directories(output_path.parent_path());
  }
  pcl::io::savePLYFile(STR(OUTPUT_POINT_CLOUD_PATH), output_point_clouds[0]);
  ASSERT_EQ(output_point_clouds[0].size(), 4096);
}

TEST(SamplingTest, farthestSamplingWithMultiplePointClouds)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>> input_point_clouds;
  std::vector<std::string> input_point_cloud_names;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> output_point_clouds;

  for (auto& dir_entry : boost::filesystem::directory_iterator(STR(INPUT_POINT_CLOUD_DIR)))
  {
    if (dir_entry.path().extension() == ".ply")
    {
      input_point_clouds.emplace_back();
      input_point_cloud_names.emplace_back(dir_entry.path().stem().c_str());
      EXPECT_NE(pcl::io::loadPLYFile<pcl::PointXYZ>(dir_entry.path().c_str(), input_point_clouds.back()), -1);
    }
  }

  if(input_point_cloud_names.size() > 1)
  {
    std::cout << "input pointcloud name pre check : " << std::endl;
    size_t same_data_num = 0;
    for(size_t i = 1; i < input_point_cloud_names.size(); ++i)
    {
      const auto &current_name = input_point_cloud_names[i];
      const auto &first_name = input_point_cloud_names[0];
      if(current_name == first_name)
      {
        ++same_data_num;
      }
    }
    std::cout << "same pointcloud name num = " << same_data_num << std::endl;
    std::cout << "pointcloud names = " << std::endl;
    for(size_t i = 1; i < input_point_cloud_names.size(); ++i)
    {
      std::cout << i << " : " << input_point_cloud_names[i] << std::endl;
    }
  }

  if(input_point_clouds.size() > 1)
  {
    std::cout << "input point cloud data pre check : " << std::endl;
    size_t same_data_num = 0;
    for (size_t i = 1; i < input_point_clouds.size(); ++i)
    {
      const auto &current_point = input_point_clouds[i].points[1];
      const auto &first_point = input_point_clouds[0].points[1];
      if(current_point.x == first_point.x &&
          current_point.y == first_point.y &&
          current_point.z == first_point.z)
      {
        ++same_data_num;
      }
    }
    std::cout << "same pointcloud num = " << same_data_num << std::endl;
  }

  farthest_sampling::samplePointCloudsCuda(input_point_clouds, output_point_clouds, 4096, true);

  if (!boost::filesystem::exists(STR(OUTPUT_POINT_CLOUD_DIR)))
  {
    boost::filesystem::create_directories(STR(OUTPUT_POINT_CLOUD_DIR));
  }

  for (std::size_t i = 0; i < output_point_clouds.size(); ++i)
  {
    ASSERT_EQ(output_point_clouds[i].size(), 4096);
    std::string output_point_cloud_name = STR(OUTPUT_POINT_CLOUD_DIR) + input_point_cloud_names[i] + "_sampled.ply";
    pcl::io::savePLYFile(output_point_cloud_name, output_point_clouds[i]);
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
