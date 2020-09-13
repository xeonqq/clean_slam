#ifndef CLEAN_SLAM_SRC_MAP_POINT_H
#define CLEAN_SLAM_SRC_MAP_POINT_H

#include <Eigen/Dense>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/signals2.hpp>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

#include "bound.h"
#include "octave_scales.h"
namespace clean_slam {

using namespace boost::signals2;

// forward decl
class KeyFrame;

struct KeyFramesCollectionCombinator {
  typedef std::vector<KeyFrame *> result_type;

  template <typename InputIterator>
  result_type operator()(InputIterator first, InputIterator last) const {
    result_type result{first, last};
    return result;
  }
};

class MapPoint {
public:
  struct OnInspectionEvent {
    using type = KeyFrame *();
    using combiner_type = KeyFramesCollectionCombinator;
  };

  MapPoint(const Eigen::Vector3d &point_3_d,
           const Eigen::Vector3d &view_direction,
           const cv::Mat &representative_descriptor,
           const BoundF &distance_bound);
  MapPoint();

  template <typename Func> boost::signals2::connection AddObserver(Func func) {
    auto c = _events.connect(func);
    Update();
    return c;
  }

  size_t NumOfObservers() const { return _events.num_slots(); }

  std::vector<KeyFrame *> Observers() const { return _events(); }
  bool operator<(const MapPoint &rhs) const;


  size_t GetId() const;

  const Eigen::Vector3d &GetPoint3D() const;

  const Eigen::Vector3d &GetViewDirection() const;

  const cv::Mat &GetRepresentativeDescriptor() const;

  bool IsObservableFromDistance(float distance) const;

  int PredictOctaveScale(float distance,
                         const OctaveScales &octave_scales) const;

  ~MapPoint();

private:
  void Update();

private:
  size_t _id;
  Eigen::Vector3d _point_3d;
  Eigen::Vector3d _view_direction; // optical center to point 3d

  // descriptor is 1 * 32 8UC1 mat, each row is a descriptor
  // in the case of ORB, the descriptor is binary, meaning, 32*8 = 256bit
  // hamming distance needs to be used to compare descriptors

  // representative descriptor has the smallest hamming distance ampng all KF
  // where this MapPoint is observed
  cv::Mat _representative_descriptor;

  // distance bound calculated from reference key frame
  BoundF _distance_bound;

  // observer
  boost::signals2::signal<OnInspectionEvent::type,
                          OnInspectionEvent::combiner_type>
      _events;
};

template <typename MapPoints>
std::vector<Eigen::Vector3d>
GetMapPointsPositions(const MapPoints &map_points) {
  std::vector<Eigen::Vector3d> map_points_positions;
  map_points_positions.reserve(boost::size(map_points));
  boost::range::transform(
      map_points, std::back_inserter(map_points_positions),
      [](const auto &map_point) { return map_point->GetPoint3D(); });
  return map_points_positions;
}

template <typename MapPoints>
cv::Mat GetMapPointsDescriptors(const MapPoints &map_points) {
  cv::Mat descriptors = cv::Mat(boost::size(map_points), 32, CV_8UC1);
  for (auto map_point : map_points | boost::adaptors::indexed()) {
    map_point.value()->GetRepresentativeDescriptor().copyTo(
        descriptors.row(map_point.index()));
  }
  return descriptors;
}
} // namespace clean_slam

namespace std {
template <> struct hash<clean_slam::MapPoint> {
  size_t operator()(const clean_slam::MapPoint &map_point) const {
    return map_point.GetId();
  }
};

} // namespace std
#endif /* MAP_POINT_H */
