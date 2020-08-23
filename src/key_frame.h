#ifndef CLEAN_SLAM_SRC_KEY_FRAME_H
#define CLEAN_SLAM_SRC_KEY_FRAME_H
#include "map_point.h"
#include <boost/signals2.hpp>
#include <opencv2/core/mat.hpp>
#include <set>
#include <vector>

namespace clean_slam {

class KeyFrame {

public:
  void AddMatchedMapPoint(MapPoint *map_point);

  size_t NumberOfMatchedMapPoints() const;

  ~KeyFrame();

private:
  cv::Mat _descriptors;
  std::vector<cv::KeyPoint> _keypoints;
  std::set<MapPoint *> _matched_map_points;
  std::vector<boost::signals2::connection> _connections;
};
} // namespace clean_slam
#endif /* KEY_FRAME_H */
