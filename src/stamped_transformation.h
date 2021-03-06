//
// Created by root on 3/14/20.
//

#ifndef CLEAN_SLAM_SRC_STAMPED_TRANSFORMATION_H_
#define CLEAN_SLAM_SRC_STAMPED_TRANSFORMATION_H_
#include "homogeneous_matrix.h"
#include <vector>

namespace clean_slam {

class StampedTransformation {
public:
  StampedTransformation() = default;
  StampedTransformation(const HomogeneousMatrix &transformation,
                        double timestamp)
      : _transformation(transformation), _timestamp(timestamp) {}

  const HomogeneousMatrix &GetTransformation() const { return _transformation; }
  double GetTimestamp() const { return _timestamp; }

private:
  HomogeneousMatrix _transformation;
  double _timestamp;
};

using CameraTrajectory = std::vector<StampedTransformation>;
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_STAMPED_TRANSFORMATION_H_
