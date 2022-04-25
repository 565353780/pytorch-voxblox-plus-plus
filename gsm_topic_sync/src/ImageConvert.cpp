#include "ImageConvert.h"

int GetType(std::string type)
{
  int ret = -1;
  if (!type.compare("mono8")) ret = CV_8UC1;  else
  if (!type.compare("8UC2" )) ret = CV_8UC2;  else
  if (!type.compare("bgr8" )) ret = CV_8UC3;  else
  if (!type.compare("8UC4" )) ret = CV_8UC4;  else
  if (!type.compare("8SC1" )) ret = CV_8SC1;  else
  if (!type.compare("8SC2" )) ret = CV_8SC2;  else
  if (!type.compare("8SC3" )) ret = CV_8SC3;  else
  if (!type.compare("8SC4" )) ret = CV_8SC4;  else
  if (!type.compare("16UC1")) ret = CV_16UC1; else
  if (!type.compare("16UC2")) ret = CV_16UC2; else
  if (!type.compare("16UC3")) ret = CV_16UC3; else
  if (!type.compare("16UC4")) ret = CV_16UC4; else
  if (!type.compare("16SC1")) ret = CV_16SC1; else
  if (!type.compare("16SC2")) ret = CV_16SC2; else
  if (!type.compare("16SC3")) ret = CV_16SC3; else
  if (!type.compare("16SC4")) ret = CV_16SC4; else
  if (!type.compare("32SC1")) ret = CV_32SC1; else
  if (!type.compare("32SC2")) ret = CV_32SC2; else
  if (!type.compare("32SC3")) ret = CV_32SC3; else
  if (!type.compare("32SC4")) ret = CV_32SC4; else
  if (!type.compare("32FC1")) ret = CV_32FC1; else
  if (!type.compare("32FC2")) ret = CV_32FC2; else
  if (!type.compare("32FC3")) ret = CV_32FC3; else
  if (!type.compare("32FC4")) ret = CV_32FC4; else
  if (!type.compare("64FC1")) ret = CV_64FC1; else
  if (!type.compare("64FC2")) ret = CV_64FC2; else
  if (!type.compare("64FC3")) ret = CV_64FC3; else
  if (!type.compare("64FC4")) ret = CV_64FC4; else;
  return ret;
}

std::string GetType(int type)
{
  std::string ret = "";
  if (type == CV_8UC1 ) ret = "mono8"; else
  if (type == CV_8UC2 ) ret = "8UC2" ; else
  if (type == CV_8UC3 ) ret = "bgr8" ; else
  if (type == CV_8UC4 ) ret = "8UC4" ; else
  if (type == CV_8SC1 ) ret = "8SC1" ; else
  if (type == CV_8SC2 ) ret = "8SC2" ; else
  if (type == CV_8SC3 ) ret = "8SC3" ; else
  if (type == CV_8SC4 ) ret = "8SC4" ; else
  if (type == CV_16UC1) ret = "16UC1"; else
  if (type == CV_16UC2) ret = "16UC2"; else
  if (type == CV_16UC3) ret = "16UC3"; else
  if (type == CV_16UC4) ret = "16UC4"; else
  if (type == CV_16SC1) ret = "16SC1"; else
  if (type == CV_16SC2) ret = "16SC2"; else
  if (type == CV_16SC3) ret = "16SC3"; else
  if (type == CV_16SC4) ret = "16SC4"; else
  if (type == CV_32SC1) ret = "32SC1"; else
  if (type == CV_32SC2) ret = "32SC2"; else
  if (type == CV_32SC3) ret = "32SC3"; else
  if (type == CV_32SC4) ret = "32SC4"; else
  if (type == CV_32FC1) ret = "32FC1"; else
  if (type == CV_32FC2) ret = "32FC2"; else
  if (type == CV_32FC3) ret = "32FC3"; else
  if (type == CV_32FC4) ret = "32FC4"; else
  if (type == CV_64FC1) ret = "64FC1"; else
  if (type == CV_64FC2) ret = "64FC2"; else
  if (type == CV_64FC3) ret = "64FC3"; else
  if (type == CV_64FC4) ret = "64FC4"; else;
  return std::move(ret);
}

int PixelSize(int type)
{
  int ret = 0;
  if (type == CV_8UC1 ) ret =  1; else
  if (type == CV_8UC2 ) ret =  2; else
  if (type == CV_8UC3 ) ret =  3; else
  if (type == CV_8UC4 ) ret =  4; else
  if (type == CV_8SC1 ) ret =  1; else
  if (type == CV_8SC2 ) ret =  2; else
  if (type == CV_8SC3 ) ret =  3; else
  if (type == CV_8SC4 ) ret =  4; else
  if (type == CV_16UC1) ret =  2; else
  if (type == CV_16UC2) ret =  4; else
  if (type == CV_16UC3) ret =  6; else
  if (type == CV_16UC4) ret =  8; else
  if (type == CV_16SC1) ret =  2; else
  if (type == CV_16SC2) ret =  4; else
  if (type == CV_16SC3) ret =  6; else
  if (type == CV_16SC4) ret =  8; else
  if (type == CV_32SC1) ret =  4; else
  if (type == CV_32SC2) ret =  8; else
  if (type == CV_32SC3) ret = 12; else
  if (type == CV_32SC4) ret = 16; else
  if (type == CV_32FC1) ret =  4; else
  if (type == CV_32FC2) ret =  8; else
  if (type == CV_32FC3) ret = 12; else
  if (type == CV_32FC4) ret = 16; else
  if (type == CV_64FC1) ret =  8; else
  if (type == CV_64FC2) ret = 16; else
  if (type == CV_64FC3) ret = 24; else
  if (type == CV_64FC4) ret = 32; else;
  return ret;
}

sensor_msgs::Image CvToRos(const cv::Mat& img)
{
  sensor_msgs::Image ret;
  ret.header.frame_id = "";
  ret.header.stamp    = ros::Time::now();
  ret.header.seq      = 0;
  ret.height          = img.rows;
  ret.width           = img.cols;
  ret.is_bigendian    = 0;
  ret.encoding        = GetType(img.type());
  ret.step            = img.cols * PixelSize(img.type());
  ret.data.resize(ret.step * img.rows);
  memcpy(ret.data.data(), img.ptr(), ret.data.size());
  return ret;
}

cv::Mat RosToCv(const sensor_msgs::Image& img)
{
  cv::Mat ret = cv::Mat(
      img.height, img.width, GetType(img.encoding));
  memcpy(ret.ptr(), img.data.data(), img.data.size());
  return ret;
}

