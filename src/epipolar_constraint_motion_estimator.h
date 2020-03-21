//
// Created by root on 3/20/20.
//

#ifndef CLEAN_SLAM_SRC_EPIPOLAR_CONSTRAINT_MOTION_ESTIMATOR_H_
#define CLEAN_SLAM_SRC_EPIPOLAR_CONSTRAINT_MOTION_ESTIMATOR_H_
#include "i_projective_transformation.h"

namespace clean_slam {

class EpipolarTransformation : public IProjectiveTransformation {
public:
  EpipolarTransformation(const cv::Mat m,
                         const std::vector<cv::Point2f> &points_previous_frame,
                         const std::vector<cv::Point2f> &points_current_frame,
                         const cv::Mat &inlier, const float reprojection_error);
  HomogeneousMatrix EstimateMotion(const cv::Mat &camera_intrinsics) override;
};

class EpipolarConstraintMotionEstimator {
public:
  EpipolarTransformation EstimateProjectiveTransformation(
      const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame) const;

  static constexpr float chi_square_threshold =
      3.84f; // one degree of freedom, 95% correction rate
  static constexpr float GetRansecThreshold() {
    return std::sqrt(chi_square_threshold);
  }

private:
  float
  CalculateSymmetricTransferError(const std::vector<cv::Point2f> &src_points,
                                  const std::vector<cv::Point2f> &dst_points,
                                  const cv::Mat &F,
                                  const cv::Mat &inlies_mask) const;
  cv::Mat CalculateEpipolarLine(const std::vector<cv::Point2f> &src_points,
                                const cv::Mat &F) const;
  cv::MatExpr
  CalculateRepojectionErrorDemoniator(const cv::Mat &epipolar_lines) const;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_EPIPOLAR_CONSTRAINT_MOTION_ESTIMATOR_H_
