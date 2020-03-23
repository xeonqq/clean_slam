//
// Created by root on 3/23/20.
//

#ifndef CLEAN_SLAM_SRC_INITIALIZER_H_
#define CLEAN_SLAM_SRC_INITIALIZER_H_
#include "camera_motion_estimator.h"
namespace clean_slam {

class Initializer {

public:
  Initializer(const cv::Mat &camera_instrinsics);
  bool Initialize(const std::vector<cv::Point2f> &points_previous_frame,
                  const std::vector<cv::Point2f> &points_current_frame);

private:
  CameraMotionEstimator _camera_motion_estimator;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_INITIALIZER_H_
