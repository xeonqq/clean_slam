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
  static KeyFrame Create(const g2o::SE3Quat &Tcw,
                         const std::vector<cv::KeyPoint> &keypoints,
                         const cv::Mat &descriptor,
                         const std::vector<Eigen::Vector3d> &matched_map_points,
                         const cv::Mat &matched_key_points_mask);

  KeyFrame() = default;
  KeyFrame(const g2o::SE3Quat &Tcw, const std::vector<cv::KeyPoint> &keypoints,
           const cv::Mat &descriptors);
  void AddMatchedMapPoint(MapPoint *map_point, size_t index);
  const g2o::SE3Quat &GetTcw() const;
  const cv::Mat &GetDescriptors() const;
  const std::vector<cv::KeyPoint> &GetKeyPoints() const;
  size_t NumKeyPoints() const;
  size_t NumberOfMatchedMapPoints() const;

  ~KeyFrame();

private:
  g2o::SE3Quat _Tcw;
  cv::Mat _descriptors;
  std::vector<cv::KeyPoint> _key_points;
  std::map<MapPoint *, size_t> _matched_map_point_to_idx;
  std::vector<boost::signals2::connection> _connections;
};
} // namespace clean_slam
#endif /* KEY_FRAME_H */
