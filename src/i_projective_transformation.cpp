//
// Created by root on 3/20/20.
//

#include "i_projective_transformation.h"
#include "cv_utils.h"
#include <cv.hpp>
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
    const cv::Mat &camera_intrinsics, cv::Mat &good_points_mask) {
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
  std::cout << "num_positive_depth_points " << num_positive_depth_points
            << std::endl;
  const float positive_ratio = 0.8f;
  //  if (num_positive_depth_points < positive_ratio * triangulated_points.rows)
  //    return num_positive_depth_points;

  // calculate parallax
  const cv::Mat &previous_camera_to_points_vec = triangulated_points;
  cv::Mat current_camera_pose = -R.t() * T;
  cv::Mat current_camera_to_points_vec;
  cv::transform(triangulated_points_current_camera_frame,
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

  // project 3d points back to image frame and calculate reprojection error
  cv::Mat reproj_previous_image_points, reproj_current_image_points;
  cv::transform(triangulated_points, reproj_previous_image_points,
                camera_intrinsics *
                    cv::Mat::eye(3, 4, camera_intrinsics.type()));
  cv::convertPointsFromHomogeneous(reproj_previous_image_points.t(),
                                   reproj_previous_image_points);
  cv::transform(triangulated_points_current_camera_frame,
                reproj_current_image_points, camera_intrinsics * extrinsics);
  cv::convertPointsFromHomogeneous(reproj_current_image_points.t(),
                                   reproj_current_image_points);

  cv::Mat reproj_error_prev_frame, reproj_error_curr_frame;

  cv::subtract(cv::Mat(_points_previous_frame).reshape(2),
               reproj_previous_image_points, reproj_error_prev_frame, _inlier);
  cv::subtract(cv::Mat(_points_current_frame).reshape(2),
               reproj_current_image_points, reproj_error_curr_frame, _inlier);

  cv::pow(reproj_error_prev_frame, 2, reproj_error_prev_frame);
  reproj_error_prev_frame = SumChannels(reproj_error_prev_frame);

  cv::pow(reproj_error_curr_frame, 2, reproj_error_curr_frame);
  reproj_error_curr_frame = SumChannels(reproj_error_curr_frame);

  //  std::cout << reproj_current_image_points.row(0) << " vs orig: "
  //  <<_points_current_frame[0]<< std::endl;
  std::cout << "prev_err: " << reproj_error_prev_frame.row(1) << std::endl;
  std::cout << "curr_err: " << reproj_error_curr_frame.row(1) << std::endl;
  return cv::countNonZero(good_points_mask);
}
} // namespace clean_slam
