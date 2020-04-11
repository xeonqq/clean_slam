//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_H_

#include "camera_motion_estimator.h"
#include "frame.h"
#include "initializer.h"
#include "optimizer.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include "stamped_transformation.h"
#include "third_party/tinyfsm.hpp"
#include "viewer.h"
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace clean_slam {

using CameraTrajectory = std::vector<StampedTransformation>;
// struct SLAM : tinyfsm::Fsm<SLAM> {};
//
// struct Init : SLAM {
//  void entry(){};
//};

class SlamCore {
public:
  SlamCore() = default;
  SlamCore(Viewer *viewer);
  void Initialize(Viewer *viewer, const cv::Mat &camera_intrinsics,
                  const cv::Mat &camera_distortion_coeffs);
  bool InitializeCameraPose(const cv::Mat &image, double timestamp);
  void TrackByMotion(const cv::Mat &image, double timestamp);
  const CameraTrajectory &GetTrajectory() const;

private:
  void DrawGoodMatches(const Frame &current_frame,
                       const std::vector<cv::DMatch> &good_matches) const;
  cv::Mat _camera_intrinsic;
  OrbExtractor _orb_extractor;
  Frame _previous_frame;
  OrbFeatureMatcher _orb_feature_matcher;
  Initializer _initializer{_camera_intrinsic};
  CameraTrajectory _trajectory;
  Viewer *_viewer;
  Optimizer _optimizer{_camera_intrinsic};
  g2o::SE3Quat _velocity;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SLAM_CORE_H_
