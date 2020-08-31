#include "key_frame.h"

namespace clean_slam {

KeyFrame::KeyFrame(const g2o::SE3Quat &Tcw,
                   const std::vector<cv::KeyPoint> &keypoints,
                   const cv::Mat &descriptors)
    : _Tcw{Tcw}, _descriptors(descriptors), _key_points(keypoints) {}

void KeyFrame::AddMatchedMapPoint(MapPoint *map_point, size_t index) {
  _connections.push_back(map_point->AddObserver([&]() { return this; }));
  _matched_map_point_to_idx.emplace(map_point, index);
}

void KeyFrame::EraseMapPoint(MapPoint *map_point) {
  _matched_map_point_to_idx.erase(map_point);
}

size_t KeyFrame::NumKeyPoints() const { return _key_points.size(); }
size_t KeyFrame::NumberOfMatchedMapPoints() const {
  return _matched_map_point_to_idx.size();
}

KeyFrame::~KeyFrame() {
  std::for_each(_connections.begin(), _connections.end(),
                [](auto &c) { c.disconnect(); });
}

const g2o::SE3Quat &KeyFrame::GetTcw() const { return _Tcw; }
const cv::Mat &KeyFrame::GetDescriptors() const { return _descriptors; }
const std::vector<cv::KeyPoint> &KeyFrame::GetKeyPoints() const {
  return _key_points;
}

} // namespace clean_slam
