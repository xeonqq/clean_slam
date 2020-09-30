//
// Created by root on 3/20/20.
//

#include "projective_transformation.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include "homography_motion_estimator.h"
#include <cv.hpp>
#include <iostream>

namespace clean_slam {

bool HasSimilarGood(const std::array<int, 4> &good_points_numbers,
                    int max_num_good_points) {
  int similar_good = 0;
  for (const auto good_points_number : good_points_numbers) {
    if (good_points_number > 0.75 * max_num_good_points) {
      ++similar_good;
    }
  }
  return similar_good > 1;
}

const float ProjectiveTransformation::GetReprojectionError() const {
  return _reprojection_error;
}

ProjectiveTransformation::ProjectiveTransformation(
    const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : _m(m), _points_previous_frame(points_previous_frame),
      _points_current_frame(points_current_frame), _inlier(inlier),
      _reprojection_error(reprojection_error) {}

PlausibleTransformation ProjectiveTransformation::ComputeTransformation(
    const cv::Mat &camera_intrinsics, const std::vector<cv::Mat> &Rs,
    const std::vector<cv::Mat> &Ts) const {
  cv::Mat projection_matrix0 =
      camera_intrinsics * cv::Mat::eye(3, 4, camera_intrinsics.type());

  std::vector<cv::Mat> projection_matrix1_candidates =
      GetProjectionMatrixCandidates(camera_intrinsics, Rs, Ts);

  std::array<int, 4> good_points_numbers{};
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
    //        std::cout << "num good points: " << good_points_numbers[i] <<
    //        std::endl;
  }
  const auto max_it =
      std::max_element(good_points_numbers.begin(), good_points_numbers.end());
  int index = std::distance(good_points_numbers.begin(), max_it);
  bool b = HasSimilarGood(good_points_numbers, *max_it);
  return PlausibleTransformation{Rs[index],         Ts[index],
                                 *max_it,           good_points_masks[index],
                                 points_3ds[index], b};
}

int ProjectiveTransformation::ValidateTriangulatedPoints(
    const cv::Mat &triangulated_points, const cv::Mat &R, const cv::Mat &T,
    const cv::Mat &camera_intrinsics, cv::Mat &good_points_mask) const {
  // triangulated points are in world frame

  cv::Mat triangulated_points_current_camera_frame;
  const cv::Mat extrinsics = ToTransformationMatrix(R, T);
  cv::transform(triangulated_points, triangulated_points_current_camera_frame,
                extrinsics);
  // positive depth for both camera positions
  good_points_mask = (triangulated_points.reshape(1).col(2) > 0) & _inlier;
  good_points_mask =
      (triangulated_points_current_camera_frame.reshape(1).col(2) > 0) &
      good_points_mask;

  // early return to save cpu, if num of positive depth points less than 80% of
  // original points
  int num_positive_depth_points = cv::countNonZero(good_points_mask);
  if (num_positive_depth_points <
      kPositivePointsRateThreshold * triangulated_points.rows)
    return num_positive_depth_points;

  // calculate parallax
  const cv::Mat &previous_camera_to_3d_points_vec = triangulated_points;
  cv::Mat current_camera_pose = -R.t() * T;
  cv::Mat current_camera_to_3d_points_vec;
  cv::transform(triangulated_points_current_camera_frame,
                current_camera_to_3d_points_vec,
                ToTransformationMatrix(cv::Mat::eye(3, 3, T.type()),
                                       -current_camera_pose));
  cv::Mat projections =
      previous_camera_to_3d_points_vec.mul(current_camera_to_3d_points_vec);

  cv::Mat norms0 = NormPoints(previous_camera_to_3d_points_vec);
  cv::Mat norms1 = NormPoints(current_camera_to_3d_points_vec);

  projections = SumChannels(projections);
  projections /= norms0.mul(norms1);

  // make sure pair of points have at least one degree of parallax
  good_points_mask = (projections < parallax_cos_threshold) & good_points_mask;

  // project 3d points back to image frame and calculate reprojection error
  cv::Mat projection_matrix_prev_frame =
      camera_intrinsics * cv::Mat::eye(3, 4, camera_intrinsics.type());
  cv::Mat projection_matrix_curr_frame = camera_intrinsics * extrinsics;

  cv::Mat reproj_error_prev_frame = Calculate3DPointsReprojectionError(
      triangulated_points, projection_matrix_prev_frame,
      _points_previous_frame);
  cv::Mat reproj_error_curr_frame = Calculate3DPointsReprojectionError(
      triangulated_points_current_camera_frame, projection_matrix_curr_frame,
      _points_current_frame);

  good_points_mask = (reproj_error_prev_frame <
                      HomographyMotionEstimator::chi_square_threshold) &
                     good_points_mask;
  good_points_mask = (reproj_error_curr_frame <
                      HomographyMotionEstimator::chi_square_threshold) &
                     good_points_mask;
  //  std::cout << reproj_current_image_points.row(0) << " vs orig: "
  //  <<_points_current_frame[0]<< std::endl;
  //  std::cout << "prev_err: " << reproj_error_prev_frame.row(1) << std::endl;
  //  std::cout << "curr_err: " << reproj_error_curr_frame.row(1) << std::endl;
  return cv::countNonZero(good_points_mask);
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

std::array<cv::Mat, 4>
GetProjectionMatrixCandidates(const cv::Mat &camera_intrinsics,
                              const cv::Mat &R1, const cv::Mat &R2,
                              const cv::Mat &T) {
  std::array<cv::Mat, 4> projection_matrix_candidates;
  projection_matrix_candidates[0] =
      camera_intrinsics * ToTransformationMatrix(R1, T);
  projection_matrix_candidates[1] =
      camera_intrinsics * ToTransformationMatrix(R1, -T);
  projection_matrix_candidates[2] =
      camera_intrinsics * ToTransformationMatrix(R2, T);
  projection_matrix_candidates[3] =
      camera_intrinsics * ToTransformationMatrix(R2, -T);
  return projection_matrix_candidates;
}
} // namespace clean_slam
