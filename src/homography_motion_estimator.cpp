//
// Created by root on 1/20/20.
//
#include "camera_motion_estimator.h"
#include "cv_utils.h"
#include "spdlog/spdlog.h"
#include <chrono>
#include <opencv2/calib3d/calib3d.hpp>

namespace clean_slam {

using namespace std::chrono;

HomographyTransformation
HomographyMotionEstimator::EstimateProjectiveTransformation(
    const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame) const {
  auto start = high_resolution_clock::now();

  cv::Mat homography_inlies;
  cv::Mat H = cv::findHomography(
      points_previous_frame, points_current_frame, homography_inlies, CV_RANSAC,
      HomographyMotionEstimator::GetRansecThreshold());
  const auto homography_average_symmetric_transfer_error = CalculateScore(
      points_previous_frame, points_current_frame, H, homography_inlies);
  auto stop = high_resolution_clock::now();

  spdlog::info("H reproject score: {}, runtime: {}",
               homography_average_symmetric_transfer_error,
               duration_cast<microseconds>(stop - start).count());

  return HomographyTransformation{H, points_previous_frame,
                                  points_current_frame, homography_inlies,
                                  homography_average_symmetric_transfer_error};
}

float HomographyMotionEstimator::CalculateScore(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &H,
    const cv::Mat &inlies_mask) const {
  const auto forward_transfer_errors =
      CalculateTransferErrors(src_points, dst_points, H, inlies_mask);
  const auto score_forward = ScoreFromChiSquareDistribution(
      forward_transfer_errors, inlies_mask, chi_square_threshold,
      chi_square_threshold);

  const auto backward_transfer_error =
      CalculateTransferErrors(dst_points, src_points, H.inv(), inlies_mask);
  const auto score_backward = ScoreFromChiSquareDistribution(
      backward_transfer_error, inlies_mask, chi_square_threshold,
      chi_square_threshold);

  return score_forward + score_backward;
}

// multi view geometry p.96
cv::Mat HomographyMotionEstimator::CalculateTransferErrors(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &m,
    const cv::Mat &inlies_mask) const {
  cv::Mat expected_dst_points;
  cv::perspectiveTransform(src_points, expected_dst_points, m);
  cv::Mat diff = ZerosLike(expected_dst_points);
  cv::subtract(expected_dst_points, cv::Mat(dst_points).reshape(2, 1), diff,
               inlies_mask.reshape(1, 1));
  cv::pow(diff, 2, diff);
  diff = diff.reshape(1, diff.cols);
  return diff.col(0) + diff.col(1);
}

HomographyTransformation::HomographyTransformation(
    const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : ProjectiveTransformation(m, points_previous_frame, points_current_frame,
                               inlier, reprojection_error) {}

PlausibleTransformation
HomographyTransformation::EstimateMotion(const cv::Mat &camera_intrinsics) {
  std::vector<cv::Mat> Rs, Ts, Normals;

  // Get R and T of previous camera pose(world) w.r.t. the current camera pose
  cv::decomposeHomographyMat(_m, camera_intrinsics, Rs, Ts, Normals);

  PlausibleTransformation plausible_transformation =
      ComputeTransformation(camera_intrinsics, Rs, Ts);

  return plausible_transformation;
}
} // namespace clean_slam
