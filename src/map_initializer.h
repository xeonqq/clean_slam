//
// Created by root on 4/25/20.
//

#ifndef CLEAN_SLAM_SRC_MAP_INITIALIZER_H_
#define CLEAN_SLAM_SRC_MAP_INITIALIZER_H_

#include <opencv2/core/mat.hpp>

#include "camera_motion_estimator.h"
#include "map.h"
#include "optimizer.h"
#include "orb_extractor.h"
#include "stamped_transformation.h"
#include "viewer.h"

namespace clean_slam {

class MapInitializer {
public:
  MapInitializer(OrbExtractor *orb_extractor,
                 const OrbFeatureMatcher *orb_feature_matcher,
                 Optimizer *optimizer, const cv::Mat &camera_intrinsic,
                 Map *map, std::vector<Frame> *frames, Viewer *viewer);

  void ProcessFirstImage(const cv::Mat &image, double timestamp);
  bool InitializeCameraPose(const cv::Mat &image, double timestamp);

private:
  OrbExtractor *_orb_extractor;
  const OrbFeatureMatcher *_orb_feature_matcher;
  Optimizer *_optimizer;
  CameraMotionEstimator _camera_motion_estimator;
  Map *_map;
  std::vector<Frame> *_frames;
  Viewer *_viewer;
  OrbFeatures _previous_orb_features;
  double _previous_timestamp;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_MAP_INITIALIZER_H_
