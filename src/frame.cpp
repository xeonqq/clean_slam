//
// Created by root on 5/12/20.
//
#include "frame.h"
namespace clean_slam {

Points3DView Frame::GetPoints3DView() const {
  return Points3DView(_map->GetPoints3D(), _map_point_indexes);
}

DescriptorsView Frame::GetDescriptorsView() const {
  return DescriptorsView(_map->GetDescriptors(), _map_point_indexes);
}
// cv::Mat Frame::GetDescriptersView() const {
//  return View<cv::Mat>
//
//  return cv::Mat(); }
} // namespace clean_slam
