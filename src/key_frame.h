#ifndef CLEAN_SLAM_SRC_KEY_FRAME_H
#define CLEAN_SLAM_SRC_KEY_FRAME_H
#include "graph.h"
#include "map_point.h"
#include "orb_extractor.h"
#include <boost/signals2.hpp>
#include <opencv2/core/mat.hpp>
#include <set>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

namespace clean_slam {
class Frame;
class OrbFeatureMatcher;

struct TriangulationResult {
  cv::Mat triangulated_points;
  std::vector<int> matched_key_points_indexes;
  std::vector<int> matched_key_points_indexes_other_frame;
};
class KeyFrame {
public:
  KeyFrame() = default;
  KeyFrame(Frame &frame, vertex_t vertex);

  const g2o::SE3Quat &GetTcw() const;
  const std::vector<cv::KeyPoint> &GetUndistortedKeyPoints() const;
  const cv::Mat &GetDescriptors() const;
  boost::select_first_range<std::map<MapPoint *, size_t>>
  GetMatchedMapPointsRng() const;
  size_t GetNumKeyPoints() const;
  size_t GetNumMatchedMapPoints() const;
  cv::Mat GetMatchedKeyPointDescriptor(MapPoint *map_point) const;

  TriangulationResult
  MatchUnmatchedKeyPoints(const OrbFeatureMatcher &matcher, KeyFrame &key_frame,
                          const cv::Mat &camera_instrinsics);
  void AddMatchedMapPoint(MapPoint *map_point, size_t index);
  void EraseMapPoint(MapPoint *map_point);
  vertex_t GetVertex() const;
  ~KeyFrame();

private:
  std::vector<boost::signals2::connection> _connections;
  Frame *_frame;
  vertex_t _vertex;
};
} // namespace clean_slam
#endif /* KEY_FRAME_H */
