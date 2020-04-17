//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_H_

#include "camera_motion_estimator.h"
#include "frame.h"
#include "initializer.h"
#include "key_frame.h"
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

class SlamCore {
public:
  SlamCore(const cv::Mat &camera_intrinsics,
           const cv::Mat &camera_distortion_coeffs, OrbExtractor *orb_extractor,
           Optimizer *optimizer, Viewer *viewer);
  bool InitializeCameraPose(const cv::Mat &image, double timestamp);
  void TrackByMotionModel(const cv::Mat &image, double timestamp);
  const CameraTrajectory &GetTrajectory() const;

private:
  void DrawGoodMatches(const Frame &current_frame,
                       const std::vector<cv::DMatch> &good_matches) const;
  cv::Mat _camera_intrinsic;
  OrbExtractor *_orb_extractor;
  Optimizer *_optimizer;
  Viewer *_viewer;

  Frame _previous_frame;
  OrbFeatureMatcher _orb_feature_matcher;
  Initializer _initializer;
  CameraTrajectory _trajectory;
  g2o::SE3Quat _velocity;

  std::vector<KeyFrame> _key_frames;
  const KeyFrame *_reference_key_frame;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SLAM_CORE_H_
