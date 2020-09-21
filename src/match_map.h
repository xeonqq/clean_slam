//
// Created by root on 9/11/20.
//

#ifndef CLEAN_SLAM_SRC_MATCH_MAP_H_
#define CLEAN_SLAM_SRC_MATCH_MAP_H_

#include <boost/range/algorithm/transform.hpp>
#include <map>
#include <opencv2/core/types.hpp>
namespace clean_slam {

class MatchMap {
public:
  void Emplace(const cv::DMatch &m);
  std::vector<cv::DMatch> ToVector() const {
    std::vector<cv::DMatch> matches;
    matches.reserve(_train_idx_to_match.size());
    boost::transform(_train_idx_to_match, std::back_inserter(matches),
                     [](const auto &pair) { return pair.second; });
    return matches;
  }

private:
  std::map<int, cv::DMatch> _train_idx_to_match;

public:
  const std::map<int, cv::DMatch> &GetTrainIdxToMatch() const;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_MATCH_MAP_H_
