//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_H_
#define CLEAN_SLAM_SRC_FRAME_H_
#include "cv_algorithms.h"
#include "key_frame.h"
#include "optimizer.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include <boost/range/adaptors.hpp>
#include <opencv2/core/core.hpp>
namespace clean_slam {
class Map;
class Frame {
public:
  static constexpr int kDescriptorDistanceThreshold = 50;
  static constexpr int kNumNearestNeighbor = 5;
  Frame() = default;

  Frame(OrbFeatures orb_features, const Map *map, const g2o::SE3Quat &Tcw,
        double timestamp)
      : _orb_features{std::move(orb_features)}, _map{map}, _Tcw{Tcw},
        _timestamp{timestamp} {}

  Frame(OrbFeatures orb_features, const Map *map, const g2o::SE3Quat &Tcw,
        double timestamp, vertex_t ref_kf)
      : _orb_features{std::move(orb_features)}, _map{map}, _Tcw{Tcw},
        _timestamp{timestamp}, _ref_kf{ref_kf} {}

  Frame(OrbFeatures orb_features, const std::vector<MapPoint *> &map_points,
        const std::vector<int> &matched_key_points_indexes, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp, vertex_t ref_kf)
      : _orb_features{std::move(orb_features)}, _map{map}, _Tcw{Tcw},
        _timestamp{timestamp}, _ref_kf{ref_kf} {
    assert(matched_key_points_indexes.size() == map_points.size());
    for (size_t i = 0; i < map_points.size(); ++i) {
      _matched_map_point_to_idx.emplace(map_points[i],
                                        matched_key_points_indexes[i]);
    }
  }

  std::vector<Eigen::Vector2d>
  ReprojectPoints3d(const std::vector<Eigen::Vector3d> &points_3d,
                    const cv::Mat &camera_intrinsic) const;

  std::vector<cv::DMatch>
  SearchByProjection(const OrbFeatureMatcher &matcher,
                     const std::vector<Eigen::Vector2d> &projected_map_points,
                     const Frame &prev_frame, const cv::Mat &mask,
                     int search_radius, const OctaveScales &octave_scales);

  size_t SearchUnmatchedKeyPointsByProjection(
      const OrbFeatureMatcher &matcher,
      const std::vector<MapPoint *> &map_points,
      const std::vector<Eigen::Vector2d> &projected_map_points,
      const std::vector<uint8_t> &map_points_octaves, int search_radius,
      const OctaveScales &octave_scales);
  void OptimizePose(OptimizerOnlyPose *optimizer_only_pose);

  std::set<vertex_t> GetKeyFramesToTrackLocalMap() const;

  std::vector<MapPoint *>
  GetMatchedMapPoints(const std::vector<cv::DMatch> &matches) const;

  const g2o::SE3Quat &GetTcw() const { return _Tcw; }
  double GetTimestamp() const { return _timestamp; }

  std::vector<uint8_t> GetMatchedMapPointsOctaves() const;
  cv::Mat GetMatchedMapPointsDescriptors() const;
  const KeyFrame &GetRefKeyFrame() const;
  size_t GetRefKeyFrameNumKeyPoints() const;
  vertex_t GetRefKfVertex() const;
  boost::select_first_range<std::map<MapPoint *, size_t>>
  GetMatchedMapPointsRng() const;
  std::vector<MapPoint *> GetMatchedMapPoints() const;

  const std::vector<cv::KeyPoint> &GetUndistortedKeyPoints() const;
  const cv::Mat &GetDescriptors() const;
  const OrbFeatures &GetOrbFeatures() const;
  size_t GetNumMatchedMapPoints() const;
  size_t GetNumKeyPoints() const;

  std::vector<cv::KeyPoint> GetMatchedKeyPoints() const;

  cv::Mat GetMatchedKeyPointDescriptor(MapPoint *map_point) const;

private:
  void SetRefKf(vertex_t kf) { _ref_kf = kf; }
  std::set<vertex_t> GetKeyFramesShareSameMapPoints() const;
  std::vector<cv::KeyPoint> GetUnmatchedKeyPoints() const;
  cv::Mat GetUnmatchedKeyPointsDescriptors() const;

private:
  friend KeyFrame;
  OrbFeatures _orb_features;
  std::map<MapPoint *, size_t> _matched_map_point_to_idx;
  const Map *_map;
  g2o::SE3Quat _Tcw;
  double _timestamp;
  vertex_t _ref_kf;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
