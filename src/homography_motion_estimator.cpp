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
  cv::Mat H = cv::findHomography(
      points_previous_frame, points_current_frame, homography_inlies, CV_RANSAC,
      HomographyMotionEstimator::GetRansecThreshold());
  const auto homography_average_symmetric_transfer_error = CalculateScore(
      points_previous_frame, points_current_frame, H, homography_inlies);
  auto stop = high_resolution_clock::now();
  std::cerr << "H reproject score:"
            << homography_average_symmetric_transfer_error
            << " runtime: " << duration_cast<microseconds>(stop - start).count()
            << '\n';

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
    : IProjectiveTransformation(m, points_previous_frame, points_current_frame,
                                inlier, reprojection_error) {}

HomogeneousMatrix
HomographyTransformation::EstimateMotion(const cv::Mat &camera_intrinsics) {
  std::vector<cv::Mat> Rs, Ts, Normals;
  HomogeneousMatrix homogeneous_matrix;

  // Get R and T of previous camera pose(world) w.r.t. the current camera pose
  cv::decomposeHomographyMat(_m, camera_intrinsics, Rs, Ts, Normals);

  cv::Mat projection_matrix0 =
      camera_intrinsics * cv::Mat::eye(3, 4, camera_intrinsics.type());

  std::vector<cv::Mat> projection_matrix1_candidates =
      GetProjectionMatrixCandidates(camera_intrinsics, Rs, Ts);

  std::array<int, 4> good_points_numbers;
  std::array<cv::Mat, 4> good_points_masks;
  std::array<cv::Mat, 4> points_3ds;

  for (size_t i = 0; i < projection_matrix1_candidates.size(); ++i) {

    cv::Mat points_3d_homo;
    cv::triangulatePoints(projection_matrix0, projection_matrix1_candidates[i],
                          _points_previous_frame, _points_current_frame,
                          points_3d_homo);
    cv::Mat points_3d_cartisian;
    cv::convertPointsFromHomogeneous(points_3d_homo.t(), points_3d_cartisian);
    // points are in rows of points_3d_cartisian, 3 channels
    good_points_numbers[i] =
        ValidateTriangulatedPoints(points_3d_cartisian, Rs[i], Ts[i],
                                   camera_intrinsics, good_points_masks[i]);
    points_3ds[i] = points_3d_cartisian;
    std::cout << "num good points: " << good_points_numbers[i] << std::endl;
    std::cout << "points 3d carti " << points_3d_cartisian.row(0) << std::endl;
  }
  const auto it =
      std::max_element(good_points_numbers.begin(), good_points_numbers.end());
  int index = std::distance(good_points_numbers.begin(), it);
  homogeneous_matrix = CreateHomogeneousMatrix(Rs[index], Ts[index]);
  return homogeneous_matrix;
}

std::vector<cv::Mat>
GetProjectionMatrixCandidates(const cv::Mat &camera_intrinsics,
                              const std::vector<cv::Mat> &Rs,
                              const std::vector<cv::Mat> &Ts) {
  std::vector<cv::Mat> projection_matrix_candidates;
  for (size_t i = 0; i < Rs.size(); ++i) {
    projection_matrix_candidates.push_back(
        camera_intrinsics * ToTransformationMatrix(Rs[i], Ts[i]));
  }
  return projection_matrix_candidates;
}

} // namespace clean_slam
