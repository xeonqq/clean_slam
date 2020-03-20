//
// Created by root on 1/20/20.
//

#ifndef CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_
#define CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_

#include <memory>
#include <opencv2/core/core.hpp>

#include "epipolar_constraint_motion_estimator.h"
#include "homogeneous_matrix.h"
#include "homography_motion_estimator.h"

namespace clean_slam {

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

float ScoreFromChiSquareDistribution(const cv::Mat &transfer_errors,
                                     const cv::Mat &inlies, float basic_score,
                                     float chi_square_threshold);
bool IsHomography(const HomographyTransformation &homography_transformation,
                  const EpipolarTransformation &epipolar_transformation);
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CAMERA_MOTION_ESTIMATOR_H_
