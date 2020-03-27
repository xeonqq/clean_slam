//
// Created by root on 3/27/20.
//

#ifndef CLEAN_SLAM_SRC_BUNDLE_ADJUSTMENT_H_
#define CLEAN_SLAM_SRC_BUNDLE_ADJUSTMENT_H_
#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/core/sparse_optimizer.h>
#include <third_party/g2o/g2o/types/se3quat.h>

namespace clean_slam {

class BundleAdjustment {
public:
  BundleAdjustment(const cv::Mat &camera_intrinsics);
  void AddPose(int id, const g2o::SE3Quat &pose, bool fixed = false);

  void AddPoint3D(int id, const Eigen::Vector3d &point_3d, bool fixed,
                  bool marginalized);

  void AddEdge(int point_3d_id, int pose_id, const Eigen::Vector2d &measurement,
               const Eigen::Matrix2d &information = Eigen::Matrix2d::Identity(),
               double delta = sqrt(5.991));

  void Optimize(int iterations = 20, bool verbose = false);

private:
  g2o::SparseOptimizer _optimizer;
  const cv::Mat &_camera_intrinsics;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_BUNDLE_ADJUSTMENT_H_
