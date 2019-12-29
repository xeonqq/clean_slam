#include "orb_extractor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace clean_slam{

OrbExtractor::OrbExtractor() : _detector{cv::ORB::create()} {}

OrbFeatures OrbExtractor::Detect(cv::Mat image) {
  OrbFeatures orb_features;
  _detector->detectAndCompute(image, cv::noArray(), orb_features.key_points,
                              orb_features.descriptors);
  return orb_features;
}
}
