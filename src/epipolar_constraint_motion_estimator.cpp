//
// Created by root on 1/20/20.
//
#include "camera_motion_estimator.h"
#include <chrono>
#include <cv.hpp>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

namespace clean_slam {

using namespace std::chrono;

EpipolarTransformation
EpipolarConstraintMotionEstimator::EstimateProjectiveTransformation(
    const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame) const {
  auto start = high_resolution_clock::now();

  cv::Mat fundamental_inlies;
  cv::Mat F = cv::findFundamentalMat(points_previous_frame,
                                     points_current_frame, fundamental_inlies);
  const auto epipolar_constraint_average_symmetric_transfer_error =
      CalculateSymmetricTransferError(
          points_previous_frame, points_current_frame, F, fundamental_inlies);
  auto stop = high_resolution_clock::now();
  std::cerr << "F reproject score:"
            << epipolar_constraint_average_symmetric_transfer_error
            << " runtime: " << duration_cast<microseconds>(stop - start).count() << '\n';

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
      epipolar_constraints.mul(epipolar_constraints)
          .mul(1 / forward_reprojection_error_demoninator);
  const auto backward_reprojection_errors =
      epipolar_constraints.mul(epipolar_constraints)
          .mul(1 / backward_reprojection_error_demoninator);
  const auto forward_score = ScoreFromChiSquareDistribution(
      forward_reprojection_errors, inlies_mask,
      HomographyMotionEstimator::chi_square_threshold_, chi_square_threshold_);
  const auto backward_score = ScoreFromChiSquareDistribution(
      forward_reprojection_errors, inlies_mask,
      HomographyMotionEstimator::chi_square_threshold_, chi_square_threshold_);
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
} // namespace clean_slam
