//
// Created by root on 5/12/20.
//
#include "frame.h"
#include <boost/range/algorithm/transform.hpp>
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
std::vector<uint8_t> Frame::GetOctaves() const {
  std::vector<uint8_t> octaves;
  octaves.reserve(_key_points.size());
  boost::range::transform(
      _key_points, std::back_inserter(octaves),
      [](const cv::KeyPoint &key_point) { return key_point.octave; });
  return octaves;
}
// cv::Mat Frame::GetDescriptersView() const {
//  return View<cv::Mat>
//
//  return cv::Mat(); }
} // namespace clean_slam
