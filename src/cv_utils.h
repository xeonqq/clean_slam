//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_CV_UTILS_H_
#define CLEAN_SLAM_SRC_CV_UTILS_H_

#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

namespace clean_slam {

struct QueryIdxs {
  std::vector<int> operator()(const std::vector<cv::DMatch> &matches) {
    std::vector<int> idxs;
    idxs.reserve(matches.size());
    std::transform(matches.begin(), matches.end(), std::back_inserter(idxs),
                   [](const auto &match) { return match.queryIdx; });
    return idxs;
  }
};

struct TrainIdxs {
  std::vector<int> operator()(const std::vector<cv::DMatch> &matches) {
    std::vector<int> idxs;
    idxs.reserve(matches.size());
    std::transform(matches.begin(), matches.end(), std::back_inserter(idxs),
                   [](const auto &match) { return match.trainIdx; });
    return idxs;
  }
};


std::string MatType2Str(int type);
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CV_UTILS_H_
