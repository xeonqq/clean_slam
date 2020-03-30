//
// Created by root on 3/23/20.
//

#ifndef CLEAN_SLAM_SRC_INITIALIZER_H_
#define CLEAN_SLAM_SRC_INITIALIZER_H_
#include "bundle_adjustment.h"
#include "camera_motion_estimator.h"
#include "octave_sigma_scales.h"
#include "orb_feature_matcher.h"

namespace clean_slam {

class Initializer {

public:
  Initializer(const cv::Mat &camera_instrinsics);
  bool Initialize(const std::vector<cv::Point2f> &points_previous_frame,
                  const std::vector<cv::Point2f> &points_current_frame);
  void RunBundleAdjustment(const KeyPointsPair &matched_key_points_pair);

  HomogeneousMatrix GetHomogeneousMatrix() const;
  cv::Mat GetTriangulatedPoints() const;
  cv::Mat GetGoodTriangulatedPoints() const;

private:
  CameraMotionEstimator _camera_motion_estimator;
  PlausibleTransformation _plausible_transformation;
  BundleAdjustment _bundle_adjustment;
  const OctaveSigmaScales _octave_sigma_scales;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_INITIALIZER_H_
