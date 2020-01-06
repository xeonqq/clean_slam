//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_H_
#define CLEAN_SLAM_SRC_FRAME_H_
#include "orb_extractor.h"
#include <opencv2/core/core.hpp>

namespace clean_slam {
class Frame {
public:
  Frame() = default;
  Frame(const cv::Mat &image, const OrbFeatures &orb_features)
      : _image(image), _orb_features(orb_features) {}

  Frame(const cv::Mat &image, OrbFeatures &&orb_features)
      : _image(image), _orb_features(std::move(orb_features)) {}

  Frame(Frame const &frame) = default;
  Frame(Frame &&frame) {
    _image = frame._image;
    _orb_features = std::move(frame._orb_features);
  }
  Frame &operator=(const Frame &frame) {
    _image = frame._image;
    _orb_features = frame._orb_features;
    return *this;
  }
  Frame &operator=(Frame &&frame) {
    _image = frame._image;
    _orb_features = std::move(frame._orb_features);
    return *this;
  }
  const cv::Mat &GetImage() const { return _image; }
  const cv::Mat GetDescriptors() const {
    return _orb_features.GetDescriptors();
  }
  const std::vector<cv::KeyPoint> &GetKeyPoints() const {
    return _orb_features.GetKeyPoints();
  }
  const std::vector<cv::KeyPoint> &GetKeyPointsUndistorted() const {
    return _orb_features.GetUndistortedKeyPoints();
  }

private:
  cv::Mat _image;
  OrbFeatures _orb_features;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
