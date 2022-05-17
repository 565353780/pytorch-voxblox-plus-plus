#include "ViewPointSaver.h"

bool BBox3D::reset()
{
  is_valid = false;
  x_min = 0;
  y_min = 0;
  z_min = 0;
  x_max = 0;
  y_max = 0;
  z_max = 0;

  return true;
}

bool BBox3D::setPosition(
    const float &new_x_min,
    const float &new_y_min,
    const float &new_z_min,
    const float &new_x_max,
    const float &new_y_max,
    const float &new_z_max)
{
  if(new_x_min > new_x_max ||
      new_y_min > new_y_max ||
      new_z_min > new_z_max)
  {
    std::cout << "BBox3D::setPosition : " << std::endl <<
      "Input :\n" <<
      "\tnew_position = [" << new_x_min << "," << new_x_max << "][" <<
      new_y_min << "," << new_y_max << "][" <<
      new_z_min << "," << new_z_max << "]" << std::endl <<
      "new position size not valid!" << std::endl;

    return false;
  }

  x_min = new_x_min;
  y_min = new_y_min;
  z_min = new_z_min;
  x_max = new_x_max;
  y_max = new_y_max;
  z_max = new_z_max;

  return true;
}

bool BBox3D::outputInfo(
    const size_t &info_level)
{
  std::string line_start = "";
  for(size_t i = 0; i < info_level; ++i)
  {
    line_start += "\t";
  }

  std::cout << line_start << "BBox3D : " << std::endl <<
    line_start << "\tposition = [" <<
    x_min << "," << x_max << "][" <<
    y_min << "," << y_max << "][" <<
    z_min << "," << z_max << "]" << std::endl <<
    line_start << "\tis_valid = " << is_valid << std::endl;

  return true;
}

bool EasyPoint3D::reset()
{
  x = 0;
  y = 0;
  z = 0;

  return true;
}

bool EasyPoint3D::outputInfo(
    const size_t &info_level)
{
  std::string line_start = "";
  for(size_t i = 0; i < info_level; ++i)
  {
    line_start += "\t";
  }

  std::cout << line_start << "EasyPoint3D : " << std::endl <<
    line_start << "\tposition = [" <<
    x << "," << y << "," << z << "]" << std::endl;

  return true;
}

bool ViewPoint3D::reset()
{
  position.reset();
  direction.reset();

  is_valid = false;

  return true;
}

bool ViewPoint3D::setPosition(
    const float &position_x,
    const float &position_y,
    const float &position_z,
    const float &direction_x,
    const float &direction_y,
    const float &direction_z)
{
  position.x = position_x;
  position.y = position_y;
  position.z = position_z;

  const float direction_length2 =
    direction_x * direction_x +
    direction_y * direction_y +
    direction_z * direction_z;

  if(direction_length2 == 0)
  {
    std::cout << "ViewPoint3D::setPosition : " << std::endl <<
      "Input :\n" <<
      "\tposition = [" << position_x << "," <<
      position_y << "," << position_z << "]" << std::endl <<
      "\tdirection = [" << direction_x << "," <<
      direction_y << "," << direction_z << "]" << std::endl <<
      "direction length = 0!" << std::endl;

    return false;
  }

  const float direction_length = std::sqrt(direction_length2);

  direction.x = direction_x / direction_length;
  direction.y = direction_y / direction_length;
  direction.z = direction_z / direction_length;

  is_valid = true;

  return true;
}

bool ViewPoint3D::outputInfo(
    const size_t &info_level)
{
  std::string line_start = "";
  for(size_t i = 0; i < info_level; ++i)
  {
    line_start += "\t";
  }

  std::cout << line_start << "ViewPoint3D : " << std::endl <<
    line_start << "\tposition :" << std::endl;
  position.outputInfo(info_level + 1);
  std::cout << line_start << "\tdirection :" << std::endl;
  direction.outputInfo(info_level + 1);
  std::cout << line_start << "\tis_valid = " << is_valid << std::endl;

  return true;
}

bool ObjectHistory::resetIdx()
{
  is_matched_ = false;
  idx_ = 0;

  return true;
}

bool ObjectHistory::reset()
{
  resetIdx();

  object_bbox_.reset();
  view_point_history_.clear();

  disappear_count_ = 0;

  return true;
}

bool ObjectHistory::setIdx(
    const size_t &idx)
{
  idx_ = idx;
  is_matched_ = true;

  return true;
}

bool ObjectHistory::setObjectBBox(
    const float &new_x_min,
    const float &new_y_min,
    const float &new_z_min,
    const float &new_x_max,
    const float &new_y_max,
    const float &new_z_max)
{
  if(!object_bbox_.setPosition(
        new_x_min, new_y_min, new_z_min,
        new_x_max, new_y_max, new_z_max))
  {
    std::cout << "ObjectHistory::setObjectBBox : " << std::endl <<
      "Input :\n" <<
      "\tnew_position = [" << new_x_min << "," << new_x_max << "][" <<
      new_y_min << "," << new_y_max << "][" <<
      new_z_min << "," << new_z_max << "]" << std::endl <<
      "setPosition for new bbox failed!" << std::endl;

    return false;
  }

  return true;
}

float ObjectHistory::getBBoxIOU(
    const BBox3D &bbox)
{
  const float bbox_iou = getBBoxIOU(bbox, object_bbox_);

  return bbox_iou;
}

float ObjectHistory::getBBoxDist(
    const BBox3D &bbox)
{
  const float center_dist2 = getBBoxCenterDist2(bbox, object_bbox_);
  const float bbox_dist2 = getBBoxDist2(bbox, object_bbox_);

  const float total_dist = center_dist2 + bbox_dist2;

  return total_dist;
}

bool ObjectHistory::addViewPoint(
    const ViewPoint3D &view_point)
{
  if(!view_point.is_valid)
  {
    std::cout << "ObjectHistory::addViewPoint : "<< std::endl <<
      "Input :\n" <<
      "\tview_point = [" << view_point.position.x << "," <<
      view_point.position.y << "," <<
      view_point.position.z << "], [" <<
      view_point.direction.x << "," <<
      view_point.direction.y << "," <<
      view_point.direction.z << "]" << std::endl <<
      "view_point not valid!" << std::endl;

    return false;
  }

  view_point_history_.emplace_back(view_point);

  return true;
}

bool ObjectHistory::isMatched()
{
  return is_matched_;
}

bool ObjectHistory::isScanFinished(
    const float &view_point_diff_max,
    const size_t &view_point_similar_time)
{
  if(view_point_diff_max < 0)
  {
    std::cout << "ObjectHistory::isScanFinished : " << std::endl <<
      "Input :\n" <<
      "\tview_point_diff_max = " << view_point_diff_max << std::endl <<
      "\tview_point_similar_time = " << view_point_similar_time << std::endl <<
      "view_point_diff_max value not valid!" << std::endl;

    return false;
  }

  if(view_point_similar_time > view_point_history_.size())
  {
    return false;
  }

  if(view_point_history_.size() == 0)
  {
    std::cout << "ObjectHistory::isScanFinished : " << std::endl <<
      "Input :\n" <<
      "\tview_point_diff_max = " << view_point_diff_max << std::endl <<
      "\tview_point_similar_time = " << view_point_similar_time << std::endl <<
      "view_point_history_ is empty! it must set when create this class!" << std::endl;

    return false;
  }

  const ViewPoint3D &latest_view_point = view_point_history_.back();

  size_t similar_view_point_count = 0;

  for(size_t i = view_point_history_.size() - 1; i >= 0; --i)
  {
    if(similar_view_point_count >= view_point_similar_time)
    {
      return true;
    }

    const float current_view_point_diff = getViewPointDist2(
        view_point_history_[i],
        latest_view_point);

    if(current_view_point_diff > view_point_diff_max)
    {
      return false;
    }

    ++similar_view_point_count;
  }

  if(similar_view_point_count < view_point_similar_time)
  {
    return false;
  }

  return true;
}

bool ObjectHistory::outputInfo(
    const size_t &info_level)
{
  std::string line_start = "";
  for(size_t i = 0; i < info_level; ++i)
  {
    line_start += "\t";
  }

  std::cout << line_start << "ObjectHistory : " << std::endl <<
    line_start << "\tidx_ = " << idx_ << std::endl <<
    line_start << "\tis_matched_ = " << is_matched_ << std::endl <<
    line_start << "\tdisappear_count_ = " << disappear_count_ << std::endl;
  object_bbox_.outputInfo(info_level + 1);
  std::cout << line_start << "\tview_point_history_ :" << std::endl;

  if(view_point_history_.size() == 0)
  {
    std::cout << "NULL" << std::endl;
  }
  else
  {
    for(ViewPoint3D &view_point : view_point_history_)
    {
      view_point.outputInfo(info_level + 1);
    }
  }

  return true;
}

float ObjectHistory::getBBoxCenterDist2(
    const BBox3D &bbox_1,
    const BBox3D &bbox_2)
{
  const float center_x_diff = std::abs(
      (bbox_1.x_min + bbox_1.x_max -
       bbox_2.x_min - bbox_2.x_max) / 2.0);

  const float center_y_diff = std::abs(
      (bbox_1.y_min + bbox_1.y_max -
       bbox_2.y_min - bbox_2.y_max) / 2.0);

  const float center_z_diff = std::abs(
      (bbox_1.z_min + bbox_1.z_max -
       bbox_2.z_min - bbox_2.z_max) / 2.0);

  const float center_dist2_error =
    center_x_diff * center_x_diff +
    center_y_diff * center_y_diff +
    center_z_diff * center_z_diff;

  return center_dist2_error;
}

float ObjectHistory::getBBoxIOU(
    const BBox3D &bbox_1,
    const BBox3D &bbox_2)
{
  if(bbox_1.x_max <= bbox_2.x_min || bbox_2.x_max <= bbox_1.x_min ||
      bbox_1.y_max <= bbox_2.y_min || bbox_2.y_max <= bbox_1.y_min ||
      bbox_1.z_max <= bbox_2.z_min || bbox_2.z_max <= bbox_1.z_min)
  {
    return 0;
  }

  const std::pair<float, float> x_min_pair = std::minmax(bbox_1.x_min, bbox_2.x_min);
  const std::pair<float, float> y_min_pair = std::minmax(bbox_1.y_min, bbox_2.y_min);
  const std::pair<float, float> z_min_pair = std::minmax(bbox_1.z_min, bbox_2.z_min);

  const std::pair<float, float> x_max_pair = std::minmax(bbox_1.x_max, bbox_2.x_max);
  const std::pair<float, float> y_max_pair = std::minmax(bbox_1.y_max, bbox_2.y_max);
  const std::pair<float, float> z_max_pair = std::minmax(bbox_1.z_max, bbox_2.z_max);

  const float x_diff_i = x_max_pair.first - x_min_pair.second;
  const float y_diff_i = y_max_pair.first - y_min_pair.second;
  const float z_diff_i = z_max_pair.first - z_min_pair.second;

  if(x_diff_i < 0 || y_diff_i < 0 || z_diff_i < 0)
  {
    std::cout << "ObjectHistory::getBBoxIOU : " << std::endl <<
      "Input :\n" <<
      "\tbbox_1 : [" << bbox_1.x_min << "," << bbox_1.x_max << "][" <<
      bbox_1.y_min << "," << bbox_1.y_max << "][" <<
      bbox_1.z_min << "," << bbox_1.z_max << "]" << std::endl <<
      "\tbbox_2 : [" << bbox_2.x_min << "," << bbox_2.x_max << "][" <<
      bbox_2.y_min << "," << bbox_2.y_max << "][" <<
      bbox_2.z_min << "," << bbox_2.z_max << "]" << std::endl <<
      "intersection size not valid!" << std::endl;

    return -1;
  }

  const float x_diff_u = x_max_pair.second - x_min_pair.first;
  const float y_diff_u = y_max_pair.second - y_min_pair.first;
  const float z_diff_u = z_max_pair.second - z_min_pair.first;

  if(x_diff_u < 0 || y_diff_u < 0 || z_diff_u < 0)
  {
    std::cout << "ObjectHistory::getBBoxIOU : " << std::endl <<
      "Input :\n" <<
      "\tbbox_1 : [" << bbox_1.x_min << "," << bbox_1.x_max << "][" <<
      bbox_1.y_min << "," << bbox_1.y_max << "][" <<
      bbox_1.z_min << "," << bbox_1.z_max << "]" << std::endl <<
      "\tbbox_2 : [" << bbox_2.x_min << "," << bbox_2.x_max << "][" <<
      bbox_2.y_min << "," << bbox_2.y_max << "][" <<
      bbox_2.z_min << "," << bbox_2.z_max << "]" << std::endl <<
      "union size not valid!" << std::endl;

    return -1;
  }

  const float volume_i = x_diff_i * y_diff_i * z_diff_i;
  const float volume_u = x_diff_u * y_diff_u * z_diff_u;

  const float iou = volume_i / volume_u;

  return iou;
}

float ObjectHistory::getBBoxDist2(
    const BBox3D &bbox_1,
    const BBox3D &bbox_2)
{
  const float x_min_diff = bbox_1.x_min - bbox_2.x_min;
  const float y_min_diff = bbox_1.y_min - bbox_2.y_min;
  const float z_min_diff = bbox_1.z_min - bbox_2.z_min;

  const float x_max_diff = bbox_1.x_max - bbox_2.x_max;
  const float y_max_diff = bbox_1.y_max - bbox_2.y_max;
  const float z_max_diff = bbox_1.z_max - bbox_2.z_max;

  const float dist2 =
    x_min_diff * x_min_diff +
    y_min_diff * y_min_diff +
    z_min_diff * z_min_diff +
    x_max_diff * x_max_diff +
    y_max_diff * y_max_diff +
    z_max_diff * z_max_diff;

  return dist2;
}

float ObjectHistory::getViewPointDist2(
    const ViewPoint3D &view_point_1,
    const ViewPoint3D &view_point_2)
{
  const float position_x_diff =
    view_point_1.position.x - view_point_2.position.x;

  const float position_y_diff =
    view_point_1.position.y - view_point_2.position.y;

  const float position_z_diff =
    view_point_1.position.z - view_point_2.position.z;

  const float direction_x_diff =
    view_point_1.direction.x - view_point_2.direction.x;

  const float direction_y_diff =
    view_point_1.direction.y - view_point_2.direction.y;

  const float direction_z_diff =
    view_point_1.direction.z - view_point_2.direction.z;

  const float total_dist =
    position_x_diff * position_x_diff +
    position_y_diff * position_y_diff +
    position_z_diff * position_z_diff +
    direction_x_diff * direction_x_diff +
    direction_y_diff * direction_y_diff +
    direction_z_diff * direction_z_diff;

  return total_dist;
}

bool ObjectSaver::reset()
{
  object_history_vec_.clear();

  return true;
}

bool ObjectSaver::resetObjectHistoryMatchState()
{
  if(object_history_vec_.size() == 0)
  {
    return true;
  }

  for(ObjectHistory &object_history : object_history_vec_)
  {
    object_history.resetIdx();
  }

  return true;
}

bool ObjectSaver::addObjectHistory(
    const size_t &object_idx,
    const BBox3D &object_bbox,
    const ViewPoint3D &object_view_point)
{
  ObjectHistory new_object_history;

  if(!new_object_history.setIdx(object_idx))
  {
    std::cout << "ObjectSaver::addObjectHistory : " << std::endl <<
      "Input :\n" <<
      "\tobject_idx = " << object_idx << std::endl <<
      "\tobject_bbox = [" <<
      object_bbox.x_min << "," << object_bbox.x_max << "][" <<
      object_bbox.y_min << "," << object_bbox.y_max << "][" <<
      object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
      "\tobject_view_point = [" << object_view_point.position.x << "," <<
      object_view_point.position.y << "," <<
      object_view_point.position.z << "][" <<
      object_view_point.direction.x << "," <<
      object_view_point.direction.y << "," <<
      object_view_point.direction.z << "]" << std::endl <<
      "setIdx failed!" << std::endl;

    return false;
  }

  if(!new_object_history.setObjectBBox(
        object_bbox.x_min, object_bbox.y_min, object_bbox.z_min,
        object_bbox.x_max, object_bbox.y_max, object_bbox.z_max))
  {
    std::cout << "ObjectSaver::addObjectHistory : " << std::endl <<
      "Input :\n" <<
      "\tobject_idx = " << object_idx << std::endl <<
      "\tobject_bbox = [" <<
      object_bbox.x_min << "," << object_bbox.x_max << "][" <<
      object_bbox.y_min << "," << object_bbox.y_max << "][" <<
      object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
      "\tobject_view_point = [" << object_view_point.position.x << "," <<
      object_view_point.position.y << "," <<
      object_view_point.position.z << "][" <<
      object_view_point.direction.x << "," <<
      object_view_point.direction.y << "," <<
      object_view_point.direction.z << "]" << std::endl <<
      "setObjectBBox failed!" << std::endl;

    return false;
  }

  if(!new_object_history.addViewPoint(object_view_point))
  {
    std::cout << "ObjectSaver::addObjectHistory : " << std::endl <<
      "Input :\n" <<
      "\tobject_idx = " << object_idx << std::endl <<
      "\tobject_bbox = [" <<
      object_bbox.x_min << "," << object_bbox.x_max << "][" <<
      object_bbox.y_min << "," << object_bbox.y_max << "][" <<
      object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
      "\tobject_view_point = [" << object_view_point.position.x << "," <<
      object_view_point.position.y << "," <<
      object_view_point.position.z << "][" <<
      object_view_point.direction.x << "," <<
      object_view_point.direction.y << "," <<
      object_view_point.direction.z << "]" << std::endl <<
      "addViewPoint failed!" << std::endl;

    return false;
  }

  object_history_vec_.emplace_back(new_object_history);

  return true;
}

bool ObjectSaver::setObjectHistory(
    const size_t &object_history_idx,
    const size_t &object_idx,
    const BBox3D &object_bbox,
    const ViewPoint3D &object_view_point)
{
  if(object_history_idx >= object_history_vec_.size())
  {
    std::cout << "ObjectSaver::setObjectHistory : " << std::endl <<
      "Input :\n" <<
      "\tobject_history_idx = " << object_history_idx << std::endl <<
      "\tobject_idx = " << object_idx << std::endl <<
      "\tobject_bbox = [" <<
      object_bbox.x_min << "," << object_bbox.x_max << "][" <<
      object_bbox.y_min << "," << object_bbox.y_max << "][" <<
      object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
      "\tobject_view_point = [" << object_view_point.position.x << "," <<
      object_view_point.position.y << "," <<
      object_view_point.position.z << "][" <<
      object_view_point.direction.x << "," <<
      object_view_point.direction.y << "," <<
      object_view_point.direction.z << "]" << std::endl <<
      "object_history_idx out of range!" << std::endl;

    return false;
  }

  ObjectHistory &object_history = object_history_vec_[object_history_idx];

  if(!object_history.setIdx(object_idx))
  {
    std::cout << "ObjectSaver::setObjectHistory : " << std::endl <<
      "Input :\n" <<
      "\tobject_history_idx = " << object_history_idx << std::endl <<
      "\tobject_idx = " << object_idx << std::endl <<
      "\tobject_bbox = [" <<
      object_bbox.x_min << "," << object_bbox.x_max << "][" <<
      object_bbox.y_min << "," << object_bbox.y_max << "][" <<
      object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
      "\tobject_view_point = [" << object_view_point.position.x << "," <<
      object_view_point.position.y << "," <<
      object_view_point.position.z << "][" <<
      object_view_point.direction.x << "," <<
      object_view_point.direction.y << "," <<
      object_view_point.direction.z << "]" << std::endl <<
      "setIdx failed!" << std::endl;

    return false;
  }

  if(!object_history.setObjectBBox(
        object_bbox.x_min, object_bbox.y_min, object_bbox.z_min,
        object_bbox.x_max, object_bbox.y_max, object_bbox.z_max))
  {
    std::cout << "ObjectSaver::setObjectHistory : " << std::endl <<
      "Input :\n" <<
      "\tobject_history_idx = " << object_history_idx << std::endl <<
      "\tobject_idx = " << object_idx << std::endl <<
      "\tobject_bbox = [" <<
      object_bbox.x_min << "," << object_bbox.x_max << "][" <<
      object_bbox.y_min << "," << object_bbox.y_max << "][" <<
      object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
      "\tobject_view_point = [" << object_view_point.position.x << "," <<
      object_view_point.position.y << "," <<
      object_view_point.position.z << "][" <<
      object_view_point.direction.x << "," <<
      object_view_point.direction.y << "," <<
      object_view_point.direction.z << "]" << std::endl <<
      "setObjectBBox failed!" << std::endl;

    return false;
  }

  if(!object_history.addViewPoint(object_view_point))
  {
    std::cout << "ObjectSaver::setObjectHistory : " << std::endl <<
      "Input :\n" <<
      "\tobject_history_idx = " << object_history_idx << std::endl <<
      "\tobject_idx = " << object_idx << std::endl <<
      "\tobject_bbox = [" <<
      object_bbox.x_min << "," << object_bbox.x_max << "][" <<
      object_bbox.y_min << "," << object_bbox.y_max << "][" <<
      object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
      "\tobject_view_point = [" << object_view_point.position.x << "," <<
      object_view_point.position.y << "," <<
      object_view_point.position.z << "][" <<
      object_view_point.direction.x << "," <<
      object_view_point.direction.y << "," <<
      object_view_point.direction.z << "]" << std::endl <<
      "addViewPoint failed!" << std::endl;

    return false;
  }

  return true;
}

bool ObjectSaver::isObjectViewPointVaild(
    const size_t &object_idx,
    const BBox3D &object_bbox,
    const ViewPoint3D &object_view_point,
    const float &view_point_diff_max,
    const size_t &view_point_similar_time)
{
  if(object_history_vec_.size() == 0)
  {
    if(!addObjectHistory(object_idx, object_bbox, object_view_point))
    {
      std::cout << "ObjectSaver::isObjectViewPointVaild : " << std::endl <<
        "Input :\n" <<
        "\tobject_idx = " << object_idx << std::endl <<
        "\tobject_bbox = [" <<
        object_bbox.x_min << "," << object_bbox.x_max << "][" <<
        object_bbox.y_min << "," << object_bbox.y_max << "][" <<
        object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
        "\tobject_view_point = [" << object_view_point.position.x << "," <<
        object_view_point.position.y << "," <<
        object_view_point.position.z << "][" <<
        object_view_point.direction.x << "," <<
        object_view_point.direction.y << "," <<
        object_view_point.direction.z << "]" << std::endl <<
        "\tview_point_diff_max = " << view_point_diff_max << std::endl <<
        "\tview_point_similar_time = " << view_point_similar_time << std::endl <<
        "addObjectHistory failed!" << std::endl;

      return false;
    }

    return true;
  }

  size_t best_match_object_history_idx;

  if(!getBestMatchObjectHistoryIdx(object_bbox, best_match_object_history_idx))
  {
    if(!addObjectHistory(object_idx, object_bbox, object_view_point))
    {
      std::cout << "ObjectSaver::isObjectViewPointVaild : " << std::endl <<
        "Input :\n" <<
        "\tobject_idx = " << object_idx << std::endl <<
        "\tobject_bbox = [" <<
        object_bbox.x_min << "," << object_bbox.x_max << "][" <<
        object_bbox.y_min << "," << object_bbox.y_max << "][" <<
        object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
        "\tobject_view_point = [" << object_view_point.position.x << "," <<
        object_view_point.position.y << "," <<
        object_view_point.position.z << "][" <<
        object_view_point.direction.x << "," <<
        object_view_point.direction.y << "," <<
        object_view_point.direction.z << "]" << std::endl <<
        "\tview_point_diff_max = " << view_point_diff_max << std::endl <<
        "\tview_point_similar_time = " << view_point_similar_time << std::endl <<
        "addObjectHistory failed!" << std::endl;

      return false;
    }

    return true;
  }

  if(!setObjectHistory(best_match_object_history_idx, object_idx, object_bbox, object_view_point))
  {
    std::cout << "ObjectSaver::isObjectViewPointVaild : " << std::endl <<
      "Input :\n" <<
      "\tobject_idx = " << object_idx << std::endl <<
      "\tobject_bbox = [" <<
      object_bbox.x_min << "," << object_bbox.x_max << "][" <<
      object_bbox.y_min << "," << object_bbox.y_max << "][" <<
      object_bbox.z_min << "," << object_bbox.z_max << "]" << std::endl <<
      "\tobject_view_point = [" << object_view_point.position.x << "," <<
      object_view_point.position.y << "," <<
      object_view_point.position.z << "][" <<
      object_view_point.direction.x << "," <<
      object_view_point.direction.y << "," <<
      object_view_point.direction.z << "]" << std::endl <<
      "\tview_point_diff_max = " << view_point_diff_max << std::endl <<
      "\tview_point_similar_time = " << view_point_similar_time << std::endl <<
      "setObjectHistory failed!" << std::endl;

    return false;
  }

  if(isObjectHistoryScanFinished(best_match_object_history_idx, view_point_diff_max, view_point_similar_time))
  {
    return false;
  }

  return true;
}

bool ObjectSaver::updateObjectHistory(
    const size_t &object_disappear_count_max)
{
  for(ObjectHistory &object_history : object_history_vec_)
  {
    if(!object_history.isMatched())
    {
      ++object_history.disappear_count_;
    }
    else
    {
      object_history.disappear_count_ = 0;
    }
  }

  if(object_disappear_count_max == 0)
  {
    std::cout << "ObjectSaver::updateObjectHistory : " << std::endl <<
      "Input :\n" <<
      "\tobject_disappear_count_max = " << object_disappear_count_max << std::endl <<
      "object_disappear_count_max must >= 1!" << std::endl;

    return false;
  }

  for(int i = object_history_vec_.size() - 1; i >= 0; --i)
  {
    ObjectHistory &object_history = object_history_vec_[i];

    if(object_history.disappear_count_ >= object_disappear_count_max)
    {
      object_history_vec_.erase(object_history_vec_.begin() + i);
    }
  }

  return true;
}

bool ObjectSaver::outputInfo(
    const size_t & info_level)
{
  std::string line_start = "";
  for(size_t i = 0; i < info_level; ++i)
  {
    line_start += "\t";
  }

  std::cout << line_start << "ObjectSaver : " << std::endl <<
    line_start << "\tobject_history_vec_ :" << std::endl;

  if(object_history_vec_.size() == 0)
  {
    std::cout << "NULL" << std::endl;
  }
  else
  {
    for(ObjectHistory &object_history : object_history_vec_)
    {
      object_history.outputInfo(info_level + 1);
    }
  }

  return true;
}

bool ObjectSaver::getBestMatchObjectHistoryIdx(
    const BBox3D &object_bbox,
    size_t &object_history_idx)
{
  object_history_idx = 0;

  if(object_history_vec_.size() == 0)
  {
    return false;
  }

  float iou_max = 0;
  float dist_min = std::numeric_limits<float>::max();

  for(size_t i = 0; i < object_history_vec_.size(); ++i)
  {
    ObjectHistory &object_history = object_history_vec_[i];

    if(object_history.isMatched())
    {
      continue;
    }

    const float current_iou = object_history.getBBoxIOU(object_bbox);

    if(current_iou == 0)
    {
      continue;
    }

    if(current_iou > iou_max)
    {
      iou_max = current_iou;
      dist_min = object_history.getBBoxDist(object_bbox);
      object_history_idx = i;
    }
    else if(current_iou == iou_max)
    {
      const float current_dist = object_history.getBBoxDist(object_bbox);
      if(current_dist < dist_min)
      {
        dist_min = current_dist;
        object_history_idx = i;
      }
    }
  }

  if(iou_max == 0)
  {
    return false;
  }

  return true;
}

bool ObjectSaver::isObjectHistoryScanFinished(
    const size_t &object_history_idx,
    const float &view_point_diff_max,
    const size_t &view_point_similar_time)
{
  if(object_history_idx >= object_history_vec_.size())
  {
    std::cout << "ObjectSaver::isObjectScanFinished : " << std::endl <<
      "Input :\n" <<
      "\tobject_history_idx = " << object_history_idx << std::endl <<
      "\tview_point_diff_max = " << view_point_diff_max << std::endl <<
      "\tview_point_similar_time = " << view_point_similar_time << std::endl <<
      "object_history_idx out of range!" << std::endl;

    return false;
  }

  return object_history_vec_[object_history_idx].isScanFinished(
      view_point_diff_max, view_point_similar_time);
}

