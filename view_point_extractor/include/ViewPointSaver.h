#ifndef VIEWPOINTSAVER_H
#define VIEWPOINTSAVER_H

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <math.h>

class BBox3D
{
public:
  BBox3D()
  {
    is_valid = false;
  }

  bool reset();

  bool setPosition(
      const float &new_x_min,
      const float &new_y_min,
      const float &new_z_min,
      const float &new_x_max,
      const float &new_y_max,
      const float &new_z_max);

  bool outputInfo(
      const size_t &info_level);

  bool is_valid;
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};

class EasyPoint3D
{
public:
  EasyPoint3D()
  {
    reset();
  }

  bool reset();

  bool outputInfo(
      const size_t &info_level);

  float x;
  float y;
  float z;
};

class ViewPoint3D
{
public:
  ViewPoint3D()
  {
    is_valid = false;
  }

  bool reset();

  bool setPosition(
      const float &position_x,
      const float &position_y,
      const float &position_z,
      const float &direction_x,
      const float &direction_y,
      const float &direction_z);

  bool outputInfo(
      const size_t &info_level);

  bool is_valid;
  EasyPoint3D position;
  EasyPoint3D direction;
};

class ObjectHistory
{
public:
  ObjectHistory()
  {
    resetIdx();

    disappear_count_ = 0;
  }

  bool resetIdx();

  bool reset();

  bool setIdx(
      const size_t &idx);

  bool setObjectBBox(
      const float &new_x_min,
      const float &new_y_min,
      const float &new_z_min,
      const float &new_x_max,
      const float &new_y_max,
      const float &new_z_max);

  float getBBoxIOU(
      const BBox3D &bbox);

  float getBBoxDist(
      const BBox3D &bbox);

  bool addViewPoint(
      const ViewPoint3D &view_point);

  bool isMatched();

  bool isScanFinished(
      const float &view_point_diff_max,
      const size_t &view_point_similar_time);

  bool outputInfo(
      const size_t &info_level);

private:
  float getBBoxCenterDist2(
      const BBox3D &bbox_1,
      const BBox3D &bbox_2);

  float getBBoxIOU(
      const BBox3D &bbox_1,
      const BBox3D &bbox_2);

  float getBBoxDist2(
      const BBox3D &bbox_1,
      const BBox3D &bbox_2);

  float getViewPointDist2(
      const ViewPoint3D &view_point_1,
      const ViewPoint3D &view_point_2);

public:
  bool is_matched_;
  size_t idx_;
  BBox3D object_bbox_;
  std::vector<ViewPoint3D> view_point_history_;

  size_t disappear_count_;
};

class ObjectSaver
{
public:
  ObjectSaver()
  {
  }

  bool reset();

  bool resetObjectHistoryMatchState();

  bool addObjectHistory(
      const size_t &object_idx,
      const BBox3D &object_bbox,
      const ViewPoint3D &object_view_point);

  bool setObjectHistory(
      const size_t &object_history_idx,
      const size_t &object_idx,
      const BBox3D &object_bbox,
      const ViewPoint3D &object_view_point);

  bool isObjectViewPointVaild(
      const size_t &object_idx,
      const BBox3D &object_bbox,
      const ViewPoint3D &object_view_point,
      const float &view_point_diff_max,
      const size_t &view_point_similar_time);

  bool updateObjectHistory(
      const size_t &object_disappear_count_max);

  bool outputInfo(
      const size_t & info_level);

private:
  bool getBestMatchObjectHistoryIdx(
      const BBox3D &object_bbox,
      size_t &object_history_idx);

  bool isObjectHistoryScanFinished(
      const size_t &object_history_idx,
      const float &view_point_diff_max,
      const size_t &view_point_similar_time);

  std::vector<ObjectHistory> object_history_vec_;
};

#endif //VIEWPOINTSAVER_H
