//
// Created by root on 1/20/20.
//

#ifndef CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_
#define CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_

#include "homogeneous_matrix.h"
#include <memory>
#include <opencv2/core/core.hpp>

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

class HomographyTransformation : public IProjectiveTransformation {
public:
  HomographyTransformation(
      const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame,
      const cv::Mat &inlier, const float reprojection_error);
  HomogeneousMatrix EstimateMotion(const cv::Mat &camera_intrinsics) override;
};

class EpipolarTransformation : public IProjectiveTransformation {
public:
  EpipolarTransformation(const cv::Mat m,
                         const std::vector<cv::Point2f> &points_previous_frame,
                         const std::vector<cv::Point2f> &points_current_frame,
                         const cv::Mat &inlier, const float reprojection_error);
  HomogeneousMatrix EstimateMotion(const cv::Mat &camera_intrinsics) override;
};

class HomographyMotionEstimator {
public:
  HomographyTransformation EstimateProjectiveTransformation(
      const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame) const;

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

class EpipolarConstraintMotionEstimator {
public:
  EpipolarTransformation EstimateProjectiveTransformation(
      const std::vector<cv::Point2f> &points_previous_frame,
      const std::vector<cv::Point2f> &points_current_frame) const;

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

class CameraMotionEstimator {
public:
  CameraMotionEstimator(const cv::Mat &camera_intrinsic);
  HomogeneousMatrix
  Estimate(const std::vector<cv::Point2f> &points_previous_frame,
           const std::vector<cv::Point2f> &points_current_frame);

private:
  HomographyMotionEstimator _homography_motion_estimator;
  EpipolarConstraintMotionEstimator _epipolar_constraint_motion_estimator;
  const cv::Mat &_camera_intrinsic;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_
