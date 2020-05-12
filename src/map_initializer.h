//
// Created by root on 4/25/20.
//

#ifndef CLEAN_SLAM_SRC_MAP_INITIALIZER_H_
#define CLEAN_SLAM_SRC_MAP_INITIALIZER_H_

#include <opencv2/core/mat.hpp>

#include "camera_motion_estimator.h"
#include "key_frame.h"
#include "map.h"
#include "optimizer.h"
#include "orb_extractor.h"
#include "stamped_transformation.h"
#include "viewer.h"
#include <boost/optional.hpp>
#include <tuple>

namespace clean_slam {

class MapInitializer {
public:
  MapInitializer(OrbExtractor *orb_extractor, Optimizer *optimizer,
                 const cv::Mat &camera_intrinsic, Map *map,
                 const OctaveScales &octave_scales, Viewer *viewer);

  void ProcessFirstImage(const cv::Mat &image, double timestamp);
  boost::optional<std::pair<Frame, Frame>>
  InitializeCameraPose(const cv::Mat &image, double timestamp);

private:
  OrbExtractor *_orb_extractor;
  Optimizer *_optimizer;
  CameraMotionEstimator _camera_motion_estimator;
  Map *_map;
  const OctaveScales &_octave_scales;
  Viewer *_viewer;
  OrbFeatureMatcher _orb_feature_matcher;
  OrbFeatures _previous_orb_features;
  double _previous_timestamp;
  Frame _frame;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_MAP_INITIALIZER_H_
