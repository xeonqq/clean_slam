#ifndef CLEAN_SLAM_SRC_MAP_POINT_H
#define CLEAN_SLAM_SRC_MAP_POINT_H

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "bound.h"
#include "key_frame.h"
#include "observer.h"

namespace clean_slam {

using ObservableMapPoint = Observable<KeyFrame>;

class MapPoint : public ObservableMapPoint {

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
};

} // namespace clean_slam
#endif /* MAP_POINT_H */
