//
// Created by root on 1/20/20.
//

#include "camera_motion_estimator.h"

namespace clean_slam {

HomogeneousMatrix CameraMotionEstimator::Estimate(
    const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame) {
  auto H_with_reproj_error =
      _homography_motion_estimator->EstimateMatAndReprojError(
          points_previous_frame, points_current_frame);
  auto F_with_reproj_error =
      _epipolar_constraint_motion_estimator->EstimateMatAndReprojError(
          points_previous_frame, points_current_frame);
  if (H_with_reproj_error.error < F_with_reproj_error.error) {
    _homography_motion_estimator->EstimateMotion(H_with_reproj_error.m);
  } else {
    _epipolar_constraint_motion_estimator->EstimateMotion(
        H_with_reproj_error.m);
  }
  return HomogeneousMatrix(cv::Mat(), cv::Mat());
}

CameraMotionEstimator::CameraMotionEstimator(const cv::Mat &camera_intrinsic)
    : _homography_motion_estimator{std::make_unique<
          HomographyMotionEstimator>()},
      _epipolar_constraint_motion_estimator{
          std::make_unique<EpipolarConstraintMotionEstimator>()},
      _camera_intrinsic(camera_intrinsic) {}

} // namespace clean_slam
