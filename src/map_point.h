#ifndef CLEAN_SLAM_SRC_MAP_POINT_H
#define CLEAN_SLAM_SRC_MAP_POINT_H

#include <Eigen/Dense>
#include <boost/signals2.hpp>
#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

#include "bound.h"
namespace clean_slam {

using namespace boost::signals2;

template <typename T> struct Average {
  typedef T result_type;

  template <typename InputIterator>
  T operator()(InputIterator first, InputIterator last) const {
    ;
  }
};

class MapPoint {
public:
  struct OnDeleteEvent {
    using type = void(MapPoint *);
  };
  struct OnUpdateEvent {
    using type = g2o::SE3Quat(MapPoint *);
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
  void Update(const cv::Mat &descriptor, const Eigen::Vector3d &view_direction);

  size_t GetId() const;

  ~MapPoint();

private:
  template <typename Event>
  const boost::signals2::signal<typename Event::type> &GetSignal() const {
    return std::get<boost::signals2::signal<typename Event::type>>(_events);
  }
  template <typename Event>
  boost::signals2::signal<typename Event::type> &GetSignal() {
    return std::get<boost::signals2::signal<typename Event::type>>(_events);
  }

  template <typename Event, typename... Args> void Signal(Args &&... args) {
    GetSignal<Event>()(std::forward<Args>(args)...);
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

  BoundF _distance_bound;

  size_t _obs_count;

  // observer
  std::tuple<boost::signals2::signal<OnDeleteEvent::type>,
             boost::signals2::signal<OnUpdateEvent::type>>
      _events;
};

// namespace std
} // namespace clean_slam
namespace std {
template <> struct hash<clean_slam::MapPoint> {
  size_t operator()(const clean_slam::MapPoint &map_point) const {
    return map_point.GetId();
  }
};
} // namespace std
#endif /* MAP_POINT_H */
