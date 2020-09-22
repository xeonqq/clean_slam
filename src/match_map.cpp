//
// Created by root on 9/11/20.
//

#include "match_map.h"
namespace clean_slam {

void MatchMap::Emplace(const cv::DMatch &m) {
  // check duplication, different query id can be matched to same train id
  if (_train_idx_to_match.find(m.trainIdx) == _train_idx_to_match.end()) {
    _train_idx_to_match.emplace(m.trainIdx, m);
  } else {
    const auto &existing_match = _train_idx_to_match[m.trainIdx];
    if (m.distance < existing_match.distance) {
      _train_idx_to_match[m.trainIdx] = m;
    }
  }
}
const std::map<int, cv::DMatch> &MatchMap::GetTrainIdxToMatch() const {
  return _train_idx_to_match;
}
std::vector<cv::DMatch>
MatchMap::Filter(const std::vector<cv::DMatch> &matches) {
  for (const auto &m : matches) {
    Emplace(m);
  }
  return ToVector();
}
std::vector<cv::DMatch> MatchMap::ToVector() const {
  std::vector<cv::DMatch> matches;
  matches.reserve(_train_idx_to_match.size());
  boost::transform(_train_idx_to_match, std::back_inserter(matches),
                   [](const auto &pair) { return pair.second; });
  return matches;
}

} // namespace clean_slam
