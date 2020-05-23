//
// Created by root on 5/3/20.
//

#include "map.h"
#include <boost/range/algorithm/transform.hpp>

namespace clean_slam {

Map::Map(std::vector<Eigen::Vector3d> &&points_3d, const cv::Mat &descriptors,
         std::vector<Bound> &&distance_bounds)
    : _points_3d(std::move(points_3d)),
      _descriptors(descriptors), _distance_bounds{std::move(distance_bounds)} {}

Map Map::Create(const g2o::SE3Quat &Tcw,
                std::vector<Eigen::Vector3d> &&points_3d,
                const cv::Mat &descriptors,
                const std::vector<cv::KeyPoint> &key_points,
                const OctaveScales &octave_scales) {
  auto distance_bounds = Calculate3DPointsDistanceBounds(
      Tcw, key_points, points_3d, octave_scales);
  return Map(std::move(points_3d), descriptors, std::move(distance_bounds));
}
void Map::Construct(const g2o::SE3Quat &Tcw,
                    std::vector<Eigen::Vector3d> &&points_3d,
                    const cv::Mat &descriptors,
                    const std::vector<cv::KeyPoint> &key_points,
                    const OctaveScales &octave_scales) {
  auto distance_bounds = Calculate3DPointsDistanceBounds(
      Tcw, key_points, points_3d, octave_scales);
  _points_3d = std::move(points_3d);
  _descriptors = descriptors;
  _distance_bounds = std::move(distance_bounds);
  _octaves.reserve(_octaves.size() + key_points.size());
  boost::range::transform(
      key_points, std::back_inserter(_octaves),
      [](const auto &key_point) { return key_point.octave; });
}

std::vector<Bound> Map::Calculate3DPointsDistanceBounds(
    const g2o::SE3Quat &Tcw, const std::vector<cv::KeyPoint> &key_points,
    std::vector<Eigen::Vector3d> &points_3d,
    const OctaveScales &octave_scales) {
  std::vector<Bound> distance_bounds;
  distance_bounds.reserve(points_3d.size());
  const auto pose_in_world = Tcw.inverse();
  for (size_t i = 0; i < points_3d.size(); ++i) {
    const auto distance = (points_3d[i] - pose_in_world.translation()).norm();
    float octave_scale = octave_scales[key_points[i].octave];
    const auto max_distance = octave_scale * distance;
    const auto min_distance =
        max_distance / octave_scales[octave_scales.size() - 1];
    distance_bounds.emplace_back(min_distance, max_distance);
  }
  return distance_bounds;
}
const std::vector<Eigen::Vector3d> &Map::GetPoints3D() const {
  return _points_3d;
}
const cv::Mat &Map::GetDescriptors() const { return _descriptors; }
const std::vector<int> &Map::GetOctaves() const { return _octaves; }

} // namespace clean_slam
