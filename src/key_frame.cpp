#include "key_frame.h"
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

namespace clean_slam {
KeyFrame::KeyFrame(const g2o::SE3Quat &Tcw, const OrbFeatures &orb_features,
                   vertex_t vertex)
    : _Tcw{Tcw}, _descriptors(orb_features.GetDescriptors()),
      _key_points(orb_features.GetUndistortedKeyPoints()), _vertex(vertex) {}

KeyFrame::KeyFrame(const g2o::SE3Quat &Tcw,
                   const std::vector<cv::KeyPoint> &keypoints,
                   const cv::Mat &descriptors, vertex_t vertex)
    : _Tcw{Tcw}, _descriptors(descriptors), _key_points(keypoints),
      _vertex(vertex) {}

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

std::vector<MapPoint *> KeyFrame::GetMatchedMapPoints() const {
  std::vector<MapPoint *> matched_map_points;
  matched_map_points.reserve(_matched_map_point_to_idx.size());
  boost::copy(_matched_map_point_to_idx | boost::adaptors::map_keys,
              std::back_inserter(matched_map_points));
  return matched_map_points;
}
vertex_t KeyFrame::GetVertex() const { return _vertex; }

} // namespace clean_slam
