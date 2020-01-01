//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_SLAM_CORE_H_
#define CLEAN_SLAM_SRC_SLAM_CORE_H_

#include "orb_extractor.h"
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace clean_slam {
class Frame {
public:
  Frame() = default;
  Frame(const cv::Mat &image, const OrbFeatures &orb_features)
      : _image(image), orb_features(orb_features) {}

  Frame(Frame const &frame) = default;
  Frame &operator=(const Frame &frame) {
    _image = frame._image;
    orb_features = frame.orb_features;
    return *this;
  }
  Frame &operator=(Frame &&frame) {
    _image = frame._image;
    orb_features = std::move(frame.orb_features);
    return *this;
  }
  const cv::Mat &GetImage() const { return _image; }
  const cv::Mat GetDescriptors() const { return orb_features.GetDescriptors(); }
  const std::vector<cv::KeyPoint> &GetKeyPoints() const {
    return orb_features.GetKeyPoints();
  }

private:
  cv::Mat _image;
  OrbFeatures orb_features;
};

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
  Frame _previous_frame;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_SLAM_CORE_H_
