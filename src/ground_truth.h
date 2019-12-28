//
// Created by root on 12/28/19.
//

#ifndef CLEAN_SLAM_SRC_GROUND_TRUTH_H_
#define CLEAN_SLAM_SRC_GROUND_TRUTH_H_

#include "third_party/g2o/g2o/types/se3quat.h"

namespace clean_slam {
class GroundTruth {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  GroundTruth(const Eigen::Vector3d &translation,
              const Eigen::Quaterniond &quaternion, double timestamp)
      : _se3_quat{quaternion, translation}, _timestamp{timestamp} {}

public:
  double GetTimestamp() const { return _timestamp; }
  const Eigen::Vector3d &GetTranslation() const {
    return _se3_quat.translation();
  }
  const Eigen::Quaterniond &GetQuaternion() const {
    return _se3_quat.rotation();
  }

private:
  g2o::SE3Quat _se3_quat;
  double _timestamp;
};

class GroundTruths : public std::vector<GroundTruth> {
public:
  GroundTruth GetGroundTruthAt(double timestamp) const;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_GROUND_TRUTH_H_
