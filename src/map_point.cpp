//
// Created by root on 8/23/20.
//
#include "map_point.h"
#include <boost/range/algorithm.hpp>

namespace clean_slam {

static size_t GenerateMapPointId() {
  static size_t id = 0;
  return id++;
}

std::vector<Eigen::Vector3d>
GetMapPointsPositions(const std::vector<MapPoint *> &map_points) {
  std::vector<Eigen::Vector3d> map_points_positions;
  map_points_positions.reserve(map_points.size());
  boost::range::transform(
      map_points, std::back_inserter(map_points_positions),
      [](const auto &map_point) { return map_point->GetPoint3D(); });
  return map_points_positions;
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

void MapPoint::Update() { _view_direction = Signal<OnUpdateEvent>(this); }

const cv::Mat &MapPoint::GetRepresentativeDescriptor() const {
  return _representative_descriptor;
}

MapPoint::~MapPoint() { Signal<OnDeleteEvent>(this); }
} // namespace clean_slam
