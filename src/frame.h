//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_H_
#define CLEAN_SLAM_SRC_FRAME_H_
#include "cv_algorithms.h"
#include "map.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include <opencv2/core/core.hpp>

namespace clean_slam {

class Frame {
public:
  static constexpr int kDescriptorDistanceThreshold = 50;
  static constexpr int kNumNearestNeighbor = 5;
  Frame() = default;
  Frame(std::vector<cv::KeyPoint> key_points,
        std::vector<MapPoint *> map_points, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp, vertex_t ref_kf)
      : _matched_key_points{std::move(key_points)},
        _matched_map_points{std::move(map_points)}, _map{map}, _Tcw{Tcw},
        _timestamp{timestamp}, _ref_kf{ref_kf} {}

  std::vector<Eigen::Vector2d>
  ReprojectPoints3d(const g2o::SE3Quat &current_pose,
                    const cv::Mat &camera_intrinsic) const;

  std::vector<cv::DMatch>
  SearchByProjection(const OrbFeatureMatcher &matcher,
                     const OrbFeatures &features,
                     const std::vector<Eigen::Vector2d> &projected_map_points,
                     const cv::Mat &mask, int search_radius,
                     const OctaveScales &octave_scales) const;

  void TrackLocalMap();

  std::vector<MapPoint *>
  GetMatchedMapPoints(const std::vector<cv::DMatch> &matches) const;

  const g2o::SE3Quat &GetTcw() const { return _Tcw; }
  double GetTimestamp() const { return _timestamp; }

  const std::vector<cv::KeyPoint> &GetKeyPoints() const;
  std::vector<uint8_t> GetOctaves() const;
  cv::Mat GetDescriptors() const;
  const KeyFrame &GetRefKeyFrame() const;
  size_t GetRefKeyFrameNumKeyPoints() const;
  vertex_t GetRefKfVertex() const;

private:
  const std::vector<cv::KeyPoint> _matched_key_points;
  const std::vector<MapPoint *> _matched_map_points;
  const Map *_map;
  g2o::SE3Quat _Tcw;
  double _timestamp;
  vertex_t _ref_kf;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
