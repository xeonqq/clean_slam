#ifndef CLEAN_SLAM_SRC_KEY_FRAME_H
#define CLEAN_SLAM_SRC_KEY_FRAME_H
#include "observer.h"
#include <opencv2/core/mat.hpp>
#include <set>
#include <vector>

namespace clean_slam {

class KeyFrame : public Observer<KeyFrame> {

  using MapPointBase = Observable<KeyFrame>;

public:
  void OnDelete(MapPointBase *map_point) {
    _matched_map_points.erase(map_point);
  }

private:
  cv::Mat _descriptors;
  std::vector<cv::KeyPoint> _keypoints;
  std::set<MapPointBase *> _matched_map_points;
};
} // namespace clean_slam
#endif /* KEY_FRAME_H */
