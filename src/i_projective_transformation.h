//
// Created by root on 3/20/20.
//

#ifndef CLEAN_SLAM_SRC_I_PROJECTIVE_TRANSFORMATION_H_
#define CLEAN_SLAM_SRC_I_PROJECTIVE_TRANSFORMATION_H_
#include "homogeneous_matrix.h"
#include <vector>

namespace clean_slam {

class IProjectiveTransformation {
public:
  virtual HomogeneousMatrix
  EstimateMotion(const cv::Mat &camera_intrinsics) = 0;
  IProjectiveTransformation(
      const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame,
      const cv::Mat &inlier, const float reprojection_error);
  const float GetReprojectionError() const;

protected:
  const cv::Mat _m; // homography matrix or fundamental matrix
  const std::vector<cv::Point2f> &_points_previous_frame;
  const std::vector<cv::Point2f> &_points_current_frame;
  const cv::Mat _inlier;
  const float _reprojection_error;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_I_PROJECTIVE_TRANSFORMATION_H_
