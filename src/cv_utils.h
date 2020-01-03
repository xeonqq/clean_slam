//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_CV_UTILS_H_
#define CLEAN_SLAM_SRC_CV_UTILS_H_

#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

namespace clean_slam {

std::vector<cv::DMatch>
RetrieveGoodMatches(const std::vector<cv::DMatch> &matches,
                    float descriptor_distance_threshold);
std::string MatType2Str(int type);
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CV_UTILS_H_
