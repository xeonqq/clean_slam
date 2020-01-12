//
// Created by root on 1/11/20.
//

#include "transfer_error.h"
#include "cv_utils.h"
#include <cv.hpp>

namespace clean_slam {
float Homography::CalculateSymmetricTransferError(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &H,
    const cv::Mat &inlies_mask) {
  const auto forward_transfer_error =
      CalculateTransferError(src_points, dst_points, H, inlies_mask);
  const auto backward_transfer_error =
      CalculateTransferError(dst_points, src_points, H.inv(), inlies_mask);
  return forward_transfer_error + backward_transfer_error;
}

float Homography::CalculateTransferError(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &m,
    const cv::Mat &inlies_mask) {
  cv::Mat expected_dst_points;
  cv::perspectiveTransform(src_points, expected_dst_points, m);
  cv::Mat diff = ZerosLike(expected_dst_points);
  cv::subtract(expected_dst_points, cv::Mat(dst_points).reshape(2, 1), diff,
               inlies_mask.reshape(1, 1));
  cv::pow(diff, 2, diff);
  auto transfer_error = cv::sum(cv::sum(diff))[0];
  //  std::cout << transfer_error << std::endl;
  return transfer_error;
}

float EpipolarConstraint::CalculateSymmetricTransferError(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &F,
    const cv::Mat &inlies_mask) {
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

  const auto symmetric_reprojection_errors =
      epipolar_constraints.mul(epipolar_constraints) /
      (forward_reprojection_error_demoninator +
       backward_reprojection_error_demoninator);

  //  std::cout << dst_points << std::endl;
  //  std::cout << epipolar_lines_forward << std::endl;
  //  std::cout << epipolar_constraints << std::endl;
  //  std::cout << symmetric_reprojection_errors << std::endl;
  cv::Mat inlies_mask_float;
  inlies_mask.convertTo(inlies_mask_float,
                        symmetric_reprojection_errors.type());
  const auto symmetric_reprojection_err =
      symmetric_reprojection_errors.dot(inlies_mask_float);
  return symmetric_reprojection_err;
}

cv::MatExpr EpipolarConstraint::CalculateRepojectionErrorDemoniator(
    const cv::Mat &epipolar_lines) {
  cv::Mat epipolar_lines_reshaped = epipolar_lines.reshape(1, 0);
  cv::Mat a = epipolar_lines_reshaped.col(0);
  cv::Mat b = epipolar_lines_reshaped.col(1);
  cv::MatExpr reprojection_error_demoninator = a.mul(a) + b.mul(b);
  return reprojection_error_demoninator;
}

cv::Mat EpipolarConstraint::CalculateEpipolarLine(
    const std::vector<cv::Point2f> &src_points, const cv::Mat &F) {
  cv::Mat epipolar_lines;
  cv::transform(src_points, epipolar_lines, F);
  epipolar_lines = epipolar_lines.reshape(0, epipolar_lines.cols);
  return epipolar_lines;
}
} // namespace clean_slam
