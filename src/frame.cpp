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

OctavesView Frame::GetOctavesView() const {
  return OctavesView(_map->GetOctaves(), _map_point_indexes);
}

const std::vector<cv::KeyPoint> &Frame::GetKeyPoints() const {
  return _key_points;
}

cv::Mat Frame::GetDescriptors() const {
  const auto view = GetDescriptorsView();
  cv::Mat descriptors = cv::Mat(view.size(), 32, CV_8UC1);
  for (size_t i = 0; i < view.size(); ++i) {
    descriptors.at<DescriptorT>(i) = view[i];
  }
  return descriptors;
}
// cv::Mat Frame::GetDescriptersView() const {
//  return View<cv::Mat>
//
//  return cv::Mat(); }
} // namespace clean_slam
