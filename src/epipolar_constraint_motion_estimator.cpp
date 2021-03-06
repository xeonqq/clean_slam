//
// Created by root on 1/20/20.
//
#include <chrono>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

#include "camera_motion_estimator.h"
#include "cv_utils.h"
#include "epipolar_constraint_motion_estimator.h"
#include "homography_motion_estimator.h"
#include "spdlog/spdlog.h"

namespace clean_slam {

using namespace std::chrono;

EpipolarTransformation
EpipolarConstraintMotionEstimator::EstimateProjectiveTransformation(
    const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame) const {
  auto start = high_resolution_clock::now();

  cv::Mat fundamental_inlies;
  cv::Mat F = cv::findFundamentalMat(
      points_previous_frame, points_current_frame, fundamental_inlies,
      cv::FM_RANSAC, EpipolarConstraintMotionEstimator::GetRansecThreshold());
  const auto epipolar_constraint_average_symmetric_transfer_error =
      CalculateSymmetricTransferError(
          points_previous_frame, points_current_frame, F, fundamental_inlies);
  auto stop = high_resolution_clock::now();

  spdlog::info("F reproject score: {}, runtime: {} ms",
               epipolar_constraint_average_symmetric_transfer_error,
               duration_cast<microseconds>(stop - start).count() / 1000.f);

  //  std::cerr << "F :" << F <<std::endl;
  return EpipolarTransformation{
      F, points_previous_frame, points_current_frame, fundamental_inlies,
      epipolar_constraint_average_symmetric_transfer_error};
}

// multi view geometry p.288
float EpipolarConstraintMotionEstimator::CalculateSymmetricTransferError(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &F,
    const cv::Mat &inlies_mask) const {
  cv::Mat epipolar_lines_forward = CalculateEpipolarLine(src_points, F);
  cv::Mat epipolar_lines_backward = CalculateEpipolarLine(dst_points, F.t());

  cv::Mat dst_points_homogeneous;
  cv::convertPointsToHomogeneous(dst_points, dst_points_homogeneous);

  cv::Mat epipolar_constraints(epipolar_lines_forward.rows, 1, CV_32FC1);
  for (int i = 0; i < epipolar_lines_forward.rows; ++i) {
    epipolar_constraints.row(i) =
        dst_points_homogeneous.row(i).dot(epipolar_lines_forward.row(i));
  }

  cv::MatExpr forward_reprojection_error_demoninator =
      CalculateRepojectionErrorDemoniator(epipolar_lines_forward);
  cv::MatExpr backward_reprojection_error_demoninator =
      CalculateRepojectionErrorDemoniator(epipolar_lines_backward);

  const auto forward_reprojection_errors =
      epipolar_constraints.mul(epipolar_constraints) /
      forward_reprojection_error_demoninator;
  const auto backward_reprojection_errors =
      epipolar_constraints.mul(epipolar_constraints) /
      backward_reprojection_error_demoninator;
  const auto forward_score = ScoreFromChiSquareDistribution(
      forward_reprojection_errors, inlies_mask,
      HomographyMotionEstimator::chi_square_threshold, chi_square_threshold);
  const auto backward_score = ScoreFromChiSquareDistribution(
      forward_reprojection_errors, inlies_mask,
      HomographyMotionEstimator::chi_square_threshold, chi_square_threshold);
  return forward_score + backward_score;
}

cv::Mat EpipolarConstraintMotionEstimator::CalculateEpipolarLine(
    const std::vector<cv::Point2f> &src_points, const cv::Mat &F) const {
  cv::Mat epipolar_lines;
  cv::transform(src_points, epipolar_lines, F);
  epipolar_lines = epipolar_lines.reshape(0, epipolar_lines.cols);
  return epipolar_lines;
}

// distance to epipolar line is regarded as error
cv::MatExpr
EpipolarConstraintMotionEstimator::CalculateRepojectionErrorDemoniator(
    const cv::Mat &epipolar_lines) const {
  cv::Mat epipolar_lines_reshaped = epipolar_lines.reshape(1);
  cv::Mat a = epipolar_lines_reshaped.col(0);
  cv::Mat b = epipolar_lines_reshaped.col(1);
  cv::MatExpr reprojection_error_demoninator = a.mul(a) + b.mul(b);
  return reprojection_error_demoninator;
}

PlausibleTransformation
EpipolarTransformation::EstimateMotion(const cv::Mat &camera_intrinsics) {
  cv::Mat R1, R2, T;
  cv::Mat essential_mat = camera_intrinsics.t() * _m * camera_intrinsics;
  cv::decomposeEssentialMat(essential_mat, R1, R2, T);
  std::vector<cv::Mat> Rs{R1, R1, R2, R2};
  std::vector<cv::Mat> Ts{T, -T, T, -T};
  PlausibleTransformation plausible_transformation =
      ComputeTransformation(camera_intrinsics, Rs, Ts);

  return plausible_transformation;
}

EpipolarTransformation::EpipolarTransformation(
    const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : ProjectiveTransformation(m, points_previous_frame, points_current_frame,
                               inlier, reprojection_error) {}

} // namespace clean_slam
