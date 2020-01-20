//
// Created by root on 1/20/20.
//

#ifndef CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_
#define CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_

#include "homogeneous_matrix.h"
#include <memory>
#include <opencv2/core/core.hpp>

namespace clean_slam {

struct MatWithReprojErr {
  const cv::Mat m; // homography matrix or fundamental matrix
  const float error;
};

class IMotionEstimator {
public:
  virtual HomogeneousMatrix
  Estimate(const std::vector<cv::Point2f> &points_previous_frame,
           const std::vector<cv::Point2f> &points_current_frame) = 0;
};

class ITwoViewsMotionEstimator {
public:
  virtual MatWithReprojErr EstimateMatAndReprojError(
      const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame) const = 0;
  virtual HomogeneousMatrix EstimateMotion(cv::Mat) const = 0;
};

class HomographyMotionEstimator : public ITwoViewsMotionEstimator {
public:
  MatWithReprojErr EstimateMatAndReprojError(
      const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame) const override;
  HomogeneousMatrix EstimateMotion(cv::Mat mat) const override;

private:
  float
  CalculateSymmetricTransferError(const std::vector<cv::Point2f> &src_points,
                                  const std::vector<cv::Point2f> &dst_points,
                                  const cv::Mat &H,
                                  const cv::Mat &inlies_mask) const;
  float CalculateTransferError(const std::vector<cv::Point2f> &src_points,
                               const std::vector<cv::Point2f> &dst_points,
                               const cv::Mat &m,
                               const cv::Mat &inlies_mask) const;
};

class EpipolarConstraintMotionEstimator : public ITwoViewsMotionEstimator {
public:
  MatWithReprojErr EstimateMatAndReprojError(
      const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame) const override;
  HomogeneousMatrix EstimateMotion(cv::Mat mat) const override;

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

class CameraMotionEstimator : public IMotionEstimator {
public:
  CameraMotionEstimator(const cv::Mat &camera_intrinsic);
  HomogeneousMatrix
  Estimate(const std::vector<cv::Point2f> &points_previous_frame,
           const std::vector<cv::Point2f> &points_current_frame) override;

private:
  std::unique_ptr<ITwoViewsMotionEstimator> _homography_motion_estimator;
  std::unique_ptr<ITwoViewsMotionEstimator>
      _epipolar_constraint_motion_estimator;
  cv::Mat _camera_intrinsic;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_
