#ifndef CLEAN_SLAM_SRC_MAP_POINT_H
#define CLEAN_SLAM_SRC_MAP_POINT_H

#include <Eigen/Dense>
#include <boost/signals2.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "bound.h"
namespace clean_slam {

class Map;

class MapPoint {
public:
  template <typename Func> boost::signals2::connection AddObserver(Func func) {
    return _events.connect(func);
  }

  size_t NumOfObservers() const;
  ;

  ~MapPoint();

private:
  Eigen::Vector3d _point_3d;
  Eigen::Vector3d _view_direction; // optical center to point 3d

  // descriptor is 1 * 32 8UC1 mat, each row is a descriptor
  // in the case of ORB, the descriptor is binary, meaning, 32*8 = 256bit
  // hamming distance needs to be used to compare descriptors

  // representative descriptor has the smallest hamming distance ampng all KF
  // where this MapPoint is observed
  cv::Mat _representative_descriptor;

  BoundF _distance_bound;

  boost::signals2::signal<void(MapPoint *)> _events; // observer
};

} // namespace clean_slam
#endif /* MAP_POINT_H */
