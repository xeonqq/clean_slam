//
// Created by root on 1/3/20.
//

#include "cv_utils.h"
#include <opencv2/core/hal/interface.h>

namespace clean_slam {

std::vector<cv::DMatch>
RetrieveGoodMatches(const std::vector<cv::DMatch> &matches,
                    float descriptor_distance_threshold) {
  // Retrieve matches have distance lower than descriptor distance threshold
  std::vector<cv::DMatch> good_matches;
  std::copy_if(matches.begin(), matches.end(), std::back_inserter(good_matches),
               [descriptor_distance_threshold](const auto &match) {
                 return match.distance < descriptor_distance_threshold;
               });
  return good_matches;
}

std::string MatType2Str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
  case CV_8U:
    r = "8U";
    break;
  case CV_8S:
    r = "8S";
    break;
  case CV_16U:
    r = "16U";
    break;
  case CV_16S:
    r = "16S";
    break;
  case CV_32S:
    r = "32S";
    break;
  case CV_32F:
    r = "32F";
    break;
  case CV_64F:
    r = "64F";
    break;
  default:
    r = "User";
    break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}
} // namespace clean_slam
