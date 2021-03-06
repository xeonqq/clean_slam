//
// Created by root on 8/23/20.
//
#include "map_point.h"
#include "cv_algorithms.h"
#include "key_frame.h"
#include <boost/range/algorithm/for_each.hpp>
namespace clean_slam {

static size_t GenerateMapPointId() {
  static size_t id = 0;
  return id++;
}

Eigen::Vector3d
AverageViewingDirection(const MapPoint &map_point,
                        const std::vector<KeyFrame *> &key_frames) {
  Eigen::Vector3d result{0, 0, 0};
  for (const auto key_frame : key_frames) {
    result += ViewingDirection(key_frame->GetTcw(), map_point.GetPoint3D());
  }
  result.normalize();
  return result;
}
cv::Mat MinDistanceDescriptor(MapPoint *map_point,
                              const std::vector<KeyFrame *> &key_frames) {
  cv::Mat min_distance_descriptor = map_point->GetRepresentativeDescriptor();
  //  for (const auto key_frame : key_frames) {
  //    int dist = cv::norm(min_distance_descriptor,
  //    key_frame->GetMatchedKeyPointDescriptor(map_point), cv::NORM_HAMMING);
  //  }
  return min_distance_descriptor;
}

MapPoint::MapPoint(const Eigen::Vector3d &point_3_d,
                   const Eigen::Vector3d &view_direction,
                   const cv::Mat &representative_descriptor,
                   const BoundF &distance_bound)
    : _id{GenerateMapPointId()}, _point_3d(point_3_d),
      _view_direction(view_direction),
      _representative_descriptor(representative_descriptor),
      _distance_bound(distance_bound) {}

MapPoint::MapPoint() : _id{GenerateMapPointId()} {}

size_t MapPoint::GetId() const { return _id; }

const Eigen::Vector3d &MapPoint::GetPoint3D() const { return _point_3d; }

const Eigen::Vector3d &MapPoint::GetViewDirection() const {
  return _view_direction;
}

bool MapPoint::operator<(const MapPoint &rhs) const { return _id < rhs._id; }

void MapPoint::Update() {
  auto key_frames = _events();
  _view_direction = AverageViewingDirection(*this, key_frames);
  // todo: update descriptor
}

const cv::Mat &MapPoint::GetRepresentativeDescriptor() const {
  return _representative_descriptor;
}

bool MapPoint::IsObservableFromDistance(float distance) const {
  return _distance_bound.IsWithIn(distance);
}

int MapPoint::PredictOctaveScale(float distance,
                                 const OctaveScales &octave_scales) const {
  return octave_scales.MapDistanceToOctaveLevel(distance, _distance_bound);
}

MapPoint::~MapPoint() {
  auto key_frames = _events();
  boost::range::for_each(
      key_frames, [&](auto key_frame) { key_frame->EraseMapPoint(this); });
}

} // namespace clean_slam
