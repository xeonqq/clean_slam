//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_H_
#define CLEAN_SLAM_SRC_FRAME_H_
#include "cv_algorithms.h"
#include "map.h"
#include "orb_extractor.h"
#include "orb_feature_matcher.h"
#include "views/elements_view.h"
#include <opencv2/core/core.hpp>

namespace clean_slam {

using Points3DView = ElementsView<std::vector<Eigen::Vector3d>>;
using DescriptorsView = ElementsView<cv::Mat, cv::Matx<uint8_t, 1, 32>>;
using OctavesView = ElementsView<std::vector<int>>;
class Frame {
public:
  static constexpr int kDescriptorDistanceThreshold = 50;
  static constexpr int kNumNearestNeighbor = 5;
  Frame() = default;
  Frame(std::vector<cv::KeyPoint> &&key_points,
        std::vector<size_t> &&map_point_indexes, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp)
      : _key_points{std::move(key_points)},
        _map_point_indexes(std::move(map_point_indexes)), _map{map}, _Tcw{Tcw},
        _timestamp{timestamp} {
    assert(_key_points.size() == _map_point_indexes.size());
  }

  Frame(const std::vector<cv::KeyPoint> &key_points,
        const std::vector<size_t> &map_point_indexes, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp)
      : _key_points{key_points},
        _map_point_indexes(map_point_indexes), _map{map}, _Tcw{Tcw},
        _timestamp{timestamp} {
    assert(_key_points.size() == _map_point_indexes.size());
  }

  std::vector<Eigen::Vector2d>
  ReprojectPoints3d(const g2o::SE3Quat &current_pose,
                    const cv::Mat &camera_intrinsic) const;

  std::vector<cv::DMatch>
  SearchByProjection(const OrbFeatureMatcher &matcher,
                     const OrbFeatures &features,
                     const std::vector<Eigen::Vector2d> &projected_map_points,
                     const cv::Mat &mask, int search_radius,
                     const OctaveScales &octave_scales) const;

  std::vector<Eigen::Vector3d>
  GetMatchedMapPoints(const std::vector<cv::DMatch> &matches) const;

  std::vector<size_t>
  GetMatchedMapPointsIds(const std::vector<cv::DMatch> &matches) const;

  const g2o::SE3Quat &GetTcw() const { return _Tcw; }
  double GetTimestamp() const { return _timestamp; }

  const std::vector<cv::KeyPoint> &GetKeyPoints() const;
  Points3DView GetPoints3DView() const;
  DescriptorsView GetDescriptorsView() const;
  std::vector<uint8_t> GetOctaves() const;
  cv::Mat GetDescriptors() const;
  size_t GetNumberOfMapPoints() const { return _map_point_indexes.size(); }

private:
  std::vector<cv::KeyPoint> _key_points;
  std::vector<size_t> _map_point_indexes;
  const Map *_map;
  g2o::SE3Quat _Tcw;
  double _timestamp;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
