//
// Created by root on 1/20/20.
//
#include "camera_motion_estimator.h"
#include "cv_utils.h"
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
  cv::Mat H = cv::findHomography(points_previous_frame, points_current_frame,
                                 homography_inlies, CV_RANSAC);
  const auto homography_average_symmetric_transfer_error =
      CalculateSymmetricTransferError(
          points_previous_frame, points_current_frame, H, homography_inlies) /
      static_cast<float>(cv::countNonZero(homography_inlies));
  auto stop = high_resolution_clock::now();
  std::cerr << "H reprojection err:"
            << homography_average_symmetric_transfer_error << " run time: "
            << duration_cast<microseconds>(stop - start).count() << '\n';

  return HomographyTransformation{H, points_previous_frame,
                                  points_current_frame, homography_inlies,
                                  homography_average_symmetric_transfer_error};
  ;
}
// HomogeneousMatrix HomographyMotionEstimator::EstimateMotion(cv::Mat H) const
// {
//      cv::decomposeHomographyMat(H, _camera_intrinsic);
//  return HomogeneousMatrix(cv::Mat(), cv::Mat());
//}

float HomographyMotionEstimator::CalculateSymmetricTransferError(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &H,
    const cv::Mat &inlies_mask) const {
  const auto forward_transfer_error =
      CalculateTransferError(src_points, dst_points, H, inlies_mask);
  const auto backward_transfer_error =
      CalculateTransferError(dst_points, src_points, H.inv(), inlies_mask);
  return forward_transfer_error + backward_transfer_error;
}

float HomographyMotionEstimator::CalculateTransferError(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &m,
    const cv::Mat &inlies_mask) const {

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
} // namespace clean_slam
