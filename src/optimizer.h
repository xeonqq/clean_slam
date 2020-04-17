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

  void NormalizeBaseLine() {
    float median_depth_inv = 1.0f / GetMedianDepthWrtWorld();
    std::for_each(
        optimized_points.begin(), optimized_points.end(),
        [&median_depth_inv](auto &point3d) { point3d *= median_depth_inv; });
    optimized_Tcw.setTranslation(optimized_Tcw.translation() *
                                 median_depth_inv);
  }

private:
  float GetMedianDepthWrtWorld() {
    std::vector<float> depths;
    depths.reserve(optimized_points.size());
    std::transform(optimized_points.begin(), optimized_points.end(),
                   std::back_inserter(depths),
                   [](const auto &point3d) { return point3d[2]; });
    const size_t median_index = static_cast<size_t>(depths.size() / 2 + 1);
    std::nth_element(depths.begin(), depths.begin() + median_index,
                     depths.end());
    return depths[median_index];
  }
};

class Optimizer {
public:
  Optimizer(const cv::Mat &camera_intrinsics, const OctaveScales &octave_scales)
      : _bundle_adjustment{camera_intrinsics}, _octave_scales{octave_scales} {}
  OptimizedResult
  Optimize(const g2o::SE3Quat &Tcw,
           const KeyPointsPair &key_points_observations,
           const std::vector<Eigen::Vector3d> &points_3d_in_world);

private:
  BundleAdjustment _bundle_adjustment;
  const OctaveScales &_octave_scales;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_OPTIMIZER_H_
