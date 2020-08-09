#ifndef CLEAN_SLAM_SRC_MAP_POINT_H
#define CLEAN_SLAM_SRC_MAP_POINT_H

#include <opencv2/core/mat.hpp>
#include <vector>

#include "bound.h"
#include "key_frame.h"
#include "observer.h"

namespace clean_slam {
class MapPoint : public Observable<KeyFrame> {

public:
  ~MapPoint() { OnDelete(); }

private:
  Eigen::Vector3d _point_3d;
  Eigen::Vector3d _view_direction; // optical center to point 3d

  // descriptors are number_points*32 8UC1 mat, each row is a descriptor
  // in the case of ORB, the descriptor is binary, meaning, 32*8 = 256bit
  // hamming distance needs to be used to compare descriptors
  cv::Mat _descriptors;

  BoundF _distance_bound;
};
} // namespace clean_slam
#endif /* MAP_POINT_H */
