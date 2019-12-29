//
// Created by root on 12/29/19.
//

#ifndef CLEAN_SLAM_SRC_ORB_EXTRACTOR_H_
#define CLEAN_SLAM_SRC_ORB_EXTRACTOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
namespace clean_slam {

struct OrbFeatures {
  std::vector<cv::KeyPoint> key_points;
  cv::Mat descriptors;
};

class OrbExtractor {
public:
  OrbExtractor();
  OrbFeatures Detect(cv::Mat image);

private:
  cv::Ptr<cv::ORB> _detector;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_ORB_EXTRACTOR_H_
