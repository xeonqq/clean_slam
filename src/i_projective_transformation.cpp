//
// Created by root on 3/20/20.
//

#include "i_projective_transformation.h"

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
} // namespace clean_slam
