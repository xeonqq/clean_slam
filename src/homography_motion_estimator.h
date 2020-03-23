//
// Created by root on 3/20/20.
//

#ifndef CLEAN_SLAM_SRC_HOMOGRAPHY_MOTION_ESTIMATOR_H_
#define CLEAN_SLAM_SRC_HOMOGRAPHY_MOTION_ESTIMATOR_H_
#include "projective_transformation.h"

namespace clean_slam {

class HomographyTransformation : public ProjectiveTransformation {
public:
  HomographyTransformation(
      const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame,
      const cv::Mat &inlier, const float reprojection_error);
  PlausibleTransformation
  EstimateMotion(const cv::Mat &camera_intrinsics) override;
};

class HomographyMotionEstimator {
public:
  HomographyTransformation EstimateProjectiveTransformation(
      const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame) const;
  static constexpr float chi_square_threshold =
      5.99f; // two degree of freedom, 95% correction rate
  static constexpr float GetRansecThreshold() {
    return std::sqrt(chi_square_threshold);
  }

private:
  float CalculateScore(const std::vector<cv::Point2f> &src_points,
                       const std::vector<cv::Point2f> &dst_points,
                       const cv::Mat &H, const cv::Mat &inlies_mask) const;
  cv::Mat CalculateTransferErrors(const std::vector<cv::Point2f> &src_points,
                                  const std::vector<cv::Point2f> &dst_points,
                                  const cv::Mat &m,
                                  const cv::Mat &inlies_mask) const;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_HOMOGRAPHY_MOTION_ESTIMATOR_H_
