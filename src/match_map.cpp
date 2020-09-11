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

} // namespace clean_slam
