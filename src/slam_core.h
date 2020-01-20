//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_H_

#include "camera_motion_estimator.h"
#include "frame.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace clean_slam {

class SlamCore {
public:
  SlamCore() = default;
  void Initialize(const cv::Mat &camera_intrinsic,
                  const cv::Mat &camera_distortion_coeffs);
  void Track(const cv::Mat &image);

private:
  cv::Mat _camera_intrinsic;
  //  cv::Mat _camera_distortion_coeffs;
  OrbExtractor _orb_extractor;
  Frame _previous_frame;
  OrbFeatureMatcher _orb_feature_matcher;
  CameraMotionEstimator _camera_motion_estimator{_camera_intrinsic};
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SLAM_CORE_H_
