//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include <opencv2/imgcodecs.hpp>

namespace clean_slam {
void SlamCore::Track(const cv::Mat image) {

  const auto orb_features = _orb_extractor.Detect(image);
  if (!_previous_image.empty()) {
  }
  _previous_image = image;

  cv::Mat out_im;
  cv::drawKeypoints(image, orb_features.key_points, out_im);
  cv::imshow("image", out_im);
  cv::waitKey(0);
}

void SlamCore::Initialize(
    const Eigen::Matrix3d &camera_intrinsic,
    const Eigen::Matrix<double, 5, 1> &camera_distortion_coeffs) {
  _camera_intrinsic = camera_intrinsic;
  _camera_distortion_coeffs = camera_distortion_coeffs;
}
} // namespace clean_slam
