//
// Created by root on 3/20/20.
//

#ifndef CLEAN_SLAM_SRC_PROJECTIVE_TRANSFORMATION_H_
#define CLEAN_SLAM_SRC_PROJECTIVE_TRANSFORMATION_H_
#include "homogeneous_matrix.h"
#include "plausible_transformation.h"
#include <vector>

namespace clean_slam {

cv::Mat
Calculate3DPointsReprojectionError(const cv::Mat &points_3d,
                                   const cv::Mat &projection_matrix,
                                   const std::vector<cv::Point2f> &points_2d);

std::vector<cv::Mat>
GetProjectionMatrixCandidates(const cv::Mat &camera_intrinsics,
                              const std::vector<cv::Mat> &Rs,
                              const std::vector<cv::Mat> &Ts);
std::array<cv::Mat, 4>
GetProjectionMatrixCandidates(const cv::Mat &camera_intrinsics,
                              const cv::Mat &R1, const cv::Mat &R2,
                              const cv::Mat &T);

class ProjectiveTransformation {
public:
  virtual PlausibleTransformation
  EstimateMotion(const cv::Mat &camera_intrinsics) = 0;
  ProjectiveTransformation(
      const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame,
      const cv::Mat &inlier, const float reprojection_error);
  const float GetReprojectionError() const;
  static constexpr float kPositivePointsRateThreshold = 0.8f;

protected:
  int ValidateTriangulatedPoints(const cv::Mat &triangulated_points,
                                 const cv::Mat &R, const cv::Mat &T,
                                 const cv::Mat &camera_intrinsics,
                                 cv::Mat &good_points_mask) const;

  PlausibleTransformation
  ComputeTransformation(const cv::Mat &camera_intrinsics,
                        const std::vector<cv::Mat> &Rs,
                        const std::vector<cv::Mat> &Ts) const;

protected:
  const cv::Mat _m; // homography matrix or fundamental matrix
  const std::vector<cv::Point2f> &_points_previous_frame;
  const std::vector<cv::Point2f> &_points_current_frame;
  const cv::Mat _inlier;
  const float _reprojection_error;
  const float parallax_cos_threshold =
      0.9998476951563913f; // at least 1 degree parallax
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_PROJECTIVE_TRANSFORMATION_H_
