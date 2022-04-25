#ifndef IMAGE_CONVERT_H
#define IMAGE_CONVERT_H

#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <memory>

int GetType(std::string type);

std::string GetType(int type);

int PixelSize(int type);

std::shared_ptr<sensor_msgs::Image> CvToRos(cv::Mat& img);

std::shared_ptr<cv::Mat> RosToCv(const sensor_msgs::Image::ConstPtr& img);

#endif // IMAGE_CONVERT_H

