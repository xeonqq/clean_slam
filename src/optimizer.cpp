//
// Created by root on 4/1/20.
//

#include "optimizer.h"
#include "cv_utils.h"
namespace clean_slam {

OptimizedResult
Optimizer::Optimize(const g2o::SE3Quat &Tcw,
                    const KeyPointsPair &key_points_observations,
                    const std::vector<Eigen::Vector3d> &points_3d_in_world) {
  _bundle_adjustment.AddPose(0, g2o::SE3Quat{}, true);
  _bundle_adjustment.AddPose(1, Tcw, false);

  const int kPoint3DInitialId = 2;
  for (size_t i = 0; i < points_3d_in_world.size(); ++i) {
    _bundle_adjustment.AddPoint3D(kPoint3DInitialId + i, points_3d_in_world[i]);
  }

  std::array<const std::vector<cv::KeyPoint> *, 2> pose_id_to_points = {
      &key_points_observations.first, &key_points_observations.second};
  for (size_t pose_id = 0; pose_id < pose_id_to_points.size(); ++pose_id) {
    for (size_t point_id = 0; point_id < pose_id_to_points[pose_id]->size();
         ++point_id) {
      const auto &key_points = *(pose_id_to_points[pose_id]);
      const auto &key_point = key_points[point_id];
      _bundle_adjustment.AddEdge(
          kPoint3DInitialId + point_id, pose_id,
          Point2fToVector2d(key_point.pt),
          Eigen::Matrix2d::Identity() *
              _octave_scales.GetOctaveInvSigma2Scales()[key_point.octave]);
    }
  }
  _bundle_adjustment.Optimize(20, true);
  //  spdlog::info("pose after bundle adjustment: {}", );
  const auto optimized_pose = _bundle_adjustment.GetOptimizedPose(1);
  std::cerr << optimized_pose << std::endl;
  std::vector<Eigen::Vector3d> optimized_points;
  optimized_points.reserve(points_3d_in_world.size());
  for (size_t point_id = 0; point_id < points_3d_in_world.size(); ++point_id) {
    optimized_points.push_back(
        _bundle_adjustment.GetOptimizedPoint(point_id + kPoint3DInitialId));
  }
  return OptimizedResult{optimized_pose, std::move(optimized_points)};
}

g2o::SE3Quat OptimizerOnlyPose::Optimize(
    const g2o::SE3Quat &Tcw,
    const std::vector<cv::KeyPoint> &key_point_observations,
    const std::vector<Eigen::Vector3d> &points_3d_in_world) {

  _bundle_adjustment.AddPose(0, Tcw, false);

  for (std::size_t i{0}; i < points_3d_in_world.size(); ++i) {
    const auto &key_point = key_point_observations[i];
    _bundle_adjustment.AddEdgeOnlyPose(
        points_3d_in_world[i], Point2fToVector2d(key_point.pt),
        Eigen::Matrix2d::Identity() *
            _octave_scales.GetOctaveInvSigma2Scales()[key_point.octave]);
  }

  _bundle_adjustment.Optimize(20, true);
  //  spdlog::info("pose after bundle adjustment: {}", );
  const auto optimized_pose = _bundle_adjustment.GetOptimizedPose(0);
  std::cerr << "pose after bundle adjustment: " << optimized_pose << std::endl;
  return optimized_pose;
}

} // namespace clean_slam
