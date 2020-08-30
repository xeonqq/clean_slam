#ifndef CLEAN_SLAM_SRC_MAP_POINT_H
#define CLEAN_SLAM_SRC_MAP_POINT_H

#include <Eigen/Dense>
#include <boost/signals2.hpp>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

#include "bound.h"
namespace clean_slam {

using namespace boost::signals2;

// forward decl
class KeyFrame;

struct ViewDirectionAverageCombinator {
  typedef Eigen::Vector3d result_type;

  template <typename InputIterator>
  result_type operator()(InputIterator first, InputIterator last) const {
    result_type result;
    for (; first != last; ++first) {
      result += *first;
    }
    result.normalize();
    return result;
  }
};

class MapPoint {
public:
  struct OnDeleteEvent {
    using type = void(MapPoint *);
    using combiner_type =
        boost::signals2::signal<OnDeleteEvent::type>::combiner_type;
  };
  struct OnUpdateEvent {
    using type = Eigen::Vector3d(MapPoint *);
    using combiner_type = ViewDirectionAverageCombinator;
  };

  MapPoint(const Eigen::Vector3d &point_3_d,
           const Eigen::Vector3d &view_direction,
           const cv::Mat &representative_descriptor,
           const BoundF &distance_bound);
  MapPoint();

  template <typename Func, typename Event>
  boost::signals2::connection AddObserver(Func func, Event) {
    return GetSignal<Event>().connect(func);
  }

  template <typename Event = OnDeleteEvent> size_t NumOfObservers() const {
    return GetSignal<Event>().num_slots();
  }

  bool operator<(const MapPoint &rhs) const;

  void Update();

  size_t GetId() const;

  const Eigen::Vector3d &GetPoint3D() const;

  const Eigen::Vector3d &GetViewDirection() const;

  ~MapPoint();

private:
  template <typename Event>
  using signal_type = boost::signals2::signal<typename Event::type,
                                              typename Event::combiner_type>;

  template <typename Event> const signal_type<Event> &GetSignal() const {
    return std::get<signal_type<Event>>(_events);
  }
  template <typename Event> signal_type<Event> &GetSignal() {
    return std::get<signal_type<Event>>(_events);
  }

  template <typename Event, typename... Args>
  typename signal_type<Event>::result_type Signal(Args &&... args) {
    return GetSignal<Event>()(std::forward<Args>(args)...);
  }

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

public:
  const cv::Mat &GetRepresentativeDescriptor() const;

private:
  // distance bound calculated from reference key frame
  BoundF _distance_bound;

  const KeyFrame *_reference_kf;
  // observer
  std::tuple<boost::signals2::signal<OnDeleteEvent::type>,
             boost::signals2::signal<OnUpdateEvent::type,
                                     OnUpdateEvent::combiner_type>>
      _events;
};

std::vector<Eigen::Vector3d>
GetMapPointsPositions(const std::vector<MapPoint *> &map_points);
} // namespace clean_slam

namespace std {
template <> struct hash<clean_slam::MapPoint> {
  size_t operator()(const clean_slam::MapPoint &map_point) const {
    return map_point.GetId();
  }
};

} // namespace std
#endif /* MAP_POINT_H */
