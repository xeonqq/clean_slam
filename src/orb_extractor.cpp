#include "orb_extractor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace clean_slam {

OrbExtractor::OrbExtractor() : _detector{cv::ORB::create()} {}

OrbFeatures OrbExtractor::Detect(cv::Mat image) {
  OrbFeatures orb_features;
  _detector->detectAndCompute(image, cv::noArray(), orb_features.GetKeyPoints(),
                              orb_features.GetDescriptors());
  return orb_features;
}

const std::vector<cv::KeyPoint> &OrbFeatures::GetKeyPoints() const {
  return _key_points;
}

const cv::Mat &OrbFeatures::GetDescriptors() const { return _descriptors; }
std::vector<cv::KeyPoint> &OrbFeatures::GetKeyPoints() { return _key_points; }
cv::Mat &OrbFeatures::GetDescriptors() { return _descriptors; }
} // namespace clean_slam
