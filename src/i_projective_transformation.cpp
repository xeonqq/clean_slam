//
// Created by root on 3/20/20.
//

#include "i_projective_transformation.h"
#include "cv_utils.h"
#include <iostream>

namespace clean_slam {

const float IProjectiveTransformation::GetReprojectionError() const {
  return _reprojection_error;
}

IProjectiveTransformation::IProjectiveTransformation(
    const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : _m(m), _points_previous_frame(points_previous_frame),
      _points_current_frame(points_current_frame), _inlier(inlier),
      _reprojection_error(reprojection_error) {}

int IProjectiveTransformation::ValidateTriangulatedPoints(
    const cv::Mat &triangulated_points, const cv::Mat &R, const cv::Mat &T,
    cv::Mat &good_points_mask) {
  // triangulated points are in world frame

  cv::Mat triangulated_points_in_current_camera_frame;
  cv::transform(triangulated_points,
                triangulated_points_in_current_camera_frame,
                ToTransformationMatrix(R, T));
  // positive depth for both camera positions
  good_points_mask = (triangulated_points.reshape(1).col(2) > 0) & _inlier;
  good_points_mask =
      (triangulated_points_in_current_camera_frame.reshape(1).col(2) > 0) &
      good_points_mask;

  // early return to save cpu, if num of positive depth points less than 80% of
  // original points
  int num_positive_depth_points = cv::countNonZero(good_points_mask);
  const float positive_ratio = 0.8f;
  if (num_positive_depth_points < positive_ratio * triangulated_points.rows)
    return num_positive_depth_points;

  // calculate parallax
  const cv::Mat &previous_camera_to_points_vec = triangulated_points;
  cv::Mat current_camera_pose = -R.t() * T;
  cv::Mat current_camera_to_points_vec;
  cv::transform(triangulated_points_in_current_camera_frame,
                current_camera_to_points_vec,
                ToTransformationMatrix(cv::Mat::eye(3, 3, T.type()),
                                       -current_camera_pose));
  cv::Mat projections =
      previous_camera_to_points_vec.mul(current_camera_to_points_vec);

  cv::Mat norms0 = NormPoints(previous_camera_to_points_vec);
  cv::Mat norms1 = NormPoints(current_camera_to_points_vec);

  projections = SumChannels(projections);
  projections /= norms0.mul(norms1);

  // make sure pair of points have at least one degree of parallax
  good_points_mask = (projections < parallax_cos_threshold) & good_points_mask;

  return cv::countNonZero(good_points_mask);
}
} // namespace clean_slam
