#include "key_frame.h"
namespace clean_slam {

KeyFrame::KeyFrame(const std::vector<cv::KeyPoint> &keypoints,
                   const cv::Mat &descriptors,
                   const std::set<MapPoint *> &matched_map_points)
    : _descriptors(descriptors), _keypoints(keypoints),
      _matched_map_points(matched_map_points) {}

void KeyFrame::AddMatchedMapPoint(MapPoint *map_point) {
  _connections.push_back(map_point->AddObserver(
      [&](MapPoint *map_point) { _matched_map_points.erase(map_point); },
      MapPoint::OnDeleteEvent{}));
  _connections.push_back(map_point->AddObserver(
      [&](MapPoint *map_point) { return _Tcw; }, MapPoint::OnUpdateEvent{}));

  _matched_map_points.insert(map_point);
}

size_t KeyFrame::NumberOfMatchedMapPoints() const {
  return _matched_map_points.size();
}

KeyFrame::~KeyFrame() {
  std::for_each(_connections.begin(), _connections.end(),
                [](auto &c) { c.disconnect(); });
}
KeyFrame
KeyFrame::Create(const g2o::SE3Quat &Tcw,
                 const std::vector<cv::KeyPoint> &keypoints,
                 const cv::Mat &descriptor,
                 const std::vector<Eigen::Vector3d> &matched_map_points) {

  return KeyFrame();
}

} // namespace clean_slam
