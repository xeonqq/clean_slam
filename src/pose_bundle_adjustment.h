//
// Created by root on 6/27/20.
//

#ifndef CLEAN_SLAM_SRC_POSE_BUNDLE_ADJUSTMENT_H_
#define CLEAN_SLAM_SRC_POSE_BUNDLE_ADJUSTMENT_H_

#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/core/sparse_optimizer.h>
#include <third_party/g2o/g2o/types/se3quat.h>

namespace clean_slam {
class PoseBundleAdjustment {
public:
  PoseBundleAdjustment(const cv::Mat &camera_intrinsics);

  void AddPose(const g2o::SE3Quat &pose);

  void AddEdge(const Eigen::Vector3d &point_3d,
               const Eigen::Vector2d &measurement,
               const Eigen::Matrix2d &information = Eigen::Matrix2d::Identity(),
               double robust_kernel_delta = sqrt(5.991));

  void Optimize(int iterations = 20, bool verbose = false);

  const g2o::SE3Quat &GetOptimizedPose() const;

private:
  g2o::SparseOptimizer _optimizer;
  cv::Mat _camera_intrinsics;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_POSE_BUNDLE_ADJUSTMENT_H_
