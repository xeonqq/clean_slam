//
// Created by root on 4/1/20.
//

#ifndef CLEAN_SLAM_SRC_OPTIMIZER_H_
#define CLEAN_SLAM_SRC_OPTIMIZER_H_
#include "bundle_adjustment.h"
#include "octave_sigma_scales.h"
#include "orb_feature_matcher.h"
namespace clean_slam {

struct OptimizedResult {
  OptimizedResult(const g2o::SE3Quat &tcw,
                  std::vector<Eigen::Vector3d> &&optimized_points)
      : optimized_Tcw(tcw), optimized_points(std::move(optimized_points)) {}
  g2o::SE3Quat optimized_Tcw;
  std::vector<Eigen::Vector3d> optimized_points;
};

class Optimizer {
public:
  Optimizer(const cv::Mat &camera_intrinsics)
      : _bundle_adjustment{camera_intrinsics} {}
  OptimizedResult
  Optimize(const g2o::SE3Quat &Tcw,
           const KeyPointsPair &key_points_observations,
           const std::vector<Eigen::Vector3d> &points_3d_in_world);

private:
  BundleAdjustment _bundle_adjustment;
  const OctaveSigmaScales _octave_sigma_scales{1.2f};
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_OPTIMIZER_H_
