//
// Created by root on 3/23/20.
//

#include "initializer.h"
#include "cv_utils.h"
#include <opencv2/core/eigen.hpp>
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {
Initializer::Initializer(const cv::Mat &camera_instrinsics)
    : _camera_motion_estimator{camera_instrinsics}, _bundle_adjustment{
                                                        camera_instrinsics} {}

bool Initializer::Initialize(
    const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame) {

  PlausibleTransformation plausible_transformation =
      _camera_motion_estimator.Estimate(points_previous_frame,
                                        points_current_frame);
  //    if (true) {
  if (plausible_transformation.IsGood()) {
    _plausible_transformation = plausible_transformation;
    return true;
  } else
    return false;
}

void Initializer::RunBundleAdjustment(
    const KeyPointsPair &matched_key_points_pair) {

  _bundle_adjustment.AddPose(0, g2o::SE3Quat{}, true);
  _bundle_adjustment.AddPose(
      1, _plausible_transformation.GetHomogeneousMatrix(), false);

  const int kPoint3DInitialId = 2;
  const auto &good_triangulated_points =
      _plausible_transformation.GetGoodTriangulatedPoints();
  for (size_t i = 0; i < good_triangulated_points.size(); ++i) {
    _bundle_adjustment.AddPoint3D(kPoint3DInitialId + i,
                                  good_triangulated_points[i]);
  }
  const auto good_key_points_prev_frame =
      FilterByMask(matched_key_points_pair.first,
                   _plausible_transformation.GetGoodPointsMask());
  const auto good_key_points_curr_frame =
      FilterByMask(matched_key_points_pair.second,
                   _plausible_transformation.GetGoodPointsMask());

  std::array<const std::vector<cv::KeyPoint> *, 2> pose_id_to_points = {
      &good_key_points_prev_frame, &good_key_points_curr_frame};
  for (size_t pose_id = 0; pose_id < pose_id_to_points.size(); ++pose_id) {
    for (size_t point_id = 0; point_id < pose_id_to_points[pose_id]->size();
         ++point_id) {
      const auto key_points = *pose_id_to_points[pose_id];
      _bundle_adjustment.AddEdge(
          kPoint3DInitialId + point_id, pose_id,
          Point2fToVector2d(key_points[point_id].pt),
          Eigen::Matrix2d::Identity() *
              _octave_sigma_scales[key_points[point_id].octave]);
    }
  }
  _bundle_adjustment.Optimize(20, true);
  //  spdlog::info("pose after bundle adjustment: {}", );
  const auto optimized_pose = _bundle_adjustment.GetOptimizedPose(1);
  std::cerr << optimized_pose << std::endl;
  std::vector<Eigen::Vector3d> optimized_points;
  for (size_t point_id = 0; point_id < good_triangulated_points.size();
       ++point_id) {
    optimized_points.push_back(
        _bundle_adjustment.GetOptimizedPoint(point_id + kPoint3DInitialId));
  }
  const auto rotation =
      optimized_pose.to_homogeneous_matrix().block<3, 3>(0, 0);
  const auto euler_angles = rotation.eulerAngles(0, 1, 2);
  //
  std::cout << "euler angle calculated: \n";
  std::cout << euler_angles << std::endl;

  //  std::cerr << optimized_points << std::endl;
  //  std::cerr << good_triangulated_points << std::endl;
}

HomogeneousMatrix Initializer::GetHomogeneousMatrix() const {
  return _plausible_transformation.GetHomogeneousMatrix();
}
cv::Mat Initializer::GetTriangulatedPoints() const {
  return _plausible_transformation.GetTriangulatedPoints();
}
const std::vector<Eigen::Vector3d> &
Initializer::GetGoodTriangulatedPoints() const {
  return _plausible_transformation.GetGoodTriangulatedPoints();
}

} // namespace clean_slam
