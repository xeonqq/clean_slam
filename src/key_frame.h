#ifndef CLEAN_SLAM_SRC_KEY_FRAME_H
#define CLEAN_SLAM_SRC_KEY_FRAME_H
#include "map_point.h"
#include <boost/signals2.hpp>
#include <opencv2/core/mat.hpp>
#include <set>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

namespace clean_slam {

class KeyFrame {

public:
  static KeyFrame
  Create(const g2o::SE3Quat &Tcw, const std::vector<cv::KeyPoint> &keypoints,
         const cv::Mat &descriptor,
         const std::vector<Eigen::Vector3d> &matched_map_points);

  KeyFrame() = default;
  KeyFrame(const std::vector<cv::KeyPoint> &keypoints,
           const cv::Mat &descriptors,
           const std::set<MapPoint *> &matched_map_points);
  void AddMatchedMapPoint(MapPoint *map_point);

  size_t NumberOfMatchedMapPoints() const;

  ~KeyFrame();

private:
  g2o::SE3Quat _Tcw;
  std::vector<cv::KeyPoint> _keypoints;
  cv::Mat _descriptors;
  std::set<MapPoint *> _matched_map_points;
  std::vector<boost::signals2::connection> _connections;
};
} // namespace clean_slam
#endif /* KEY_FRAME_H */
