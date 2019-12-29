//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_H_

#include "orb_extractor.h"
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace clean_slam {
class SlamCore {
public:
  void Track(const cv::Mat image);
  SlamCore() = default;
  void Initialize(const Eigen::Matrix3d &camera_intrinsic,
                  const Eigen::Matrix<double, 5, 1> &camera_distortion_coeffs);

private:
  Eigen::Matrix3d _camera_intrinsic;
  Eigen::Matrix<double, 5, 1> _camera_distortion_coeffs;
  OrbExtractor _orb_extractor;
  cv::Mat _previous_image;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SLAM_CORE_H_
