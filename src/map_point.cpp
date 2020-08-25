//
// Created by root on 8/23/20.
//
#include "map_point.h"
namespace clean_slam {

static size_t GenerateMapPointId() {
  static size_t id = 0;
  return id++;
}

MapPoint::MapPoint(const Eigen::Vector3d &point_3_d,
                   const Eigen::Vector3d &view_direction,
                   const cv::Mat &representative_descriptor,
                   const BoundF &distance_bound)
    : _id{GenerateMapPointId()}, _point_3d(point_3_d),
      _view_direction(view_direction),
      _representative_descriptor(representative_descriptor),
      _distance_bound(distance_bound), _obs_count{1} {}

MapPoint::MapPoint() : _id{GenerateMapPointId()} {}

size_t MapPoint::GetId() const { return _id; }

const Eigen::Vector3d &MapPoint::GetPoint3D() const { return _point_3d; }

const Eigen::Vector3d &MapPoint::GetViewDirection() const {
  return _view_direction;
}
bool MapPoint::operator<(const MapPoint &rhs) const { return _id < rhs._id; }

void MapPoint::Update() { _view_direction = Signal<OnUpdateEvent>(this); }

MapPoint::~MapPoint() { Signal<OnDeleteEvent>(this); }
} // namespace clean_slam
