//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_H_
#define CLEAN_SLAM_SRC_FRAME_H_
#include "cv_algorithms.h"
#include "key_frame.h"
#include "map.h"
#include "optimizer.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include <opencv2/core/core.hpp>

namespace clean_slam {

class Frame {
public:
  static constexpr int kDescriptorDistanceThreshold = 50;
  static constexpr int kNumNearestNeighbor = 5;
  Frame() = default;
  Frame(OrbFeatures orb_features, const Map *map, const g2o::SE3Quat &Tcw,
        double timestamp, vertex_t ref_kf)
      : _orb_features{std::move(orb_features)}, _map{map}, _Tcw{Tcw},
        _timestamp{timestamp}, _ref_kf{ref_kf} {}

  Frame(std::vector<cv::KeyPoint> key_points,
        std::vector<MapPoint *> map_points, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp, vertex_t ref_kf)
      : _matched_key_points{std::move(key_points)},
        _matched_map_points{std::move(map_points)}, _map{map}, _Tcw{Tcw},
        _timestamp{timestamp}, _ref_kf{ref_kf} {}

  std::vector<Eigen::Vector2d>
  ReprojectPoints3d(const std::vector<Eigen::Vector3d> &points_3d,
                    const cv::Mat &camera_intrinsic) const;

  std::vector<cv::DMatch>
  SearchByProjection(const OrbFeatureMatcher &matcher,
                     const std::vector<Eigen::Vector2d> &projected_map_points,
                     const Frame &prev_frame, const cv::Mat &mask,
                     int search_radius, const OctaveScales &octave_scales);

  void OptimizePose(OptimizerOnlyPose *optimizer_only_pose);

  std::set<vertex_t> GetKeyFramesToTrackLocalMap() const;

  std::vector<MapPoint *>
  GetMatchedMapPoints(const std::vector<cv::DMatch> &matches) const;

  std::vector<cv::KeyPoint>
  GetMatchedKeyPoints(const std::vector<cv::DMatch> &matches) const;

  const g2o::SE3Quat &GetTcw() const { return _Tcw; }
  double GetTimestamp() const { return _timestamp; }

  const std::vector<cv::KeyPoint> &GetKeyPoints() const;
  std::vector<uint8_t> GetMatchedMapPointsOctaves() const;
  cv::Mat GetMatchedMapPointsDescriptors() const;
  const KeyFrame &GetRefKeyFrame() const;
  size_t GetRefKeyFrameNumKeyPoints() const;
  vertex_t GetRefKfVertex() const;
  const std::vector<MapPoint *> &GetMatchedMapPoints() const;

  const std::vector<cv::KeyPoint> &GetUndistortedKeyPoints() const;
  const OrbFeatures &GetOrbFeatures() const;

private:
  std::set<vertex_t> GetKeyFramesShareSameMapPoints() const;

private:
  OrbFeatures _orb_features;
  std::vector<cv::KeyPoint> _matched_key_points;
  std::vector<MapPoint *> _matched_map_points;
  const Map *_map;
  g2o::SE3Quat _Tcw;
  double _timestamp;
  vertex_t _ref_kf;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
