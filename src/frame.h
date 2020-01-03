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
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
