//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_CV_UTILS_H_
#define CLEAN_SLAM_SRC_CV_UTILS_H_

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>
namespace clean_slam {

std::string MatType2Str(int type);

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

template <typename T, typename V = uint8_t>
std::vector<T> FilterByMask(const std::vector<T> &vec, cv::Mat mask) {
  assert(vec.size() == mask.rows);
  assert(mask.cols == 1);
  std::vector<T> result;
  for (size_t i = 0; i < mask.rows; ++i) {
    if (mask.at<V>(i) == 1) {
      result.push_back(vec[i]);
    }
  }
  return result;
}

template <class T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &values) {
  os << "[";
  for (const auto &v : values) {
    os << "; " << v;
  }
  os << "]";
  return os;
}
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CV_UTILS_H_
