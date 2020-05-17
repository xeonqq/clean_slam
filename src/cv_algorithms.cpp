//
// Created by root on 4/18/20.
//
#include "cv_algorithms.h"

namespace clean_slam {

g2o::SE3Quat GetVelocity(const g2o::SE3Quat &Tcw_current,
                         const g2o::SE3Quat &Tcw_prev) {
  return Tcw_current * Tcw_prev.inverse();
}

std::vector<Eigen::Vector2d>
ReprojectPoints3d(const std::vector<Eigen::Vector3d> &points_3d,
                  const g2o::SE3Quat &current_pose,
                  const cv::Mat &camera_intrinsic) {
  std::vector<Eigen::Vector2d> points_reprojected;
  points_reprojected.resize(points_3d.size());
  ReprojectPoints3d(points_3d, points_reprojected.begin(), current_pose,
                    camera_intrinsic);
  return points_reprojected;
}
std::vector<std::pair<size_t, size_t>>
SearchByProjection(const OrbFeatures &features,
                   const std::vector<Eigen::Vector2d> &projected_map_points,
                   const DescriptorsView &map_points_descriptors,
                   const std::vector<bool> &mask, int search_radius) {
  /*
   * We have orb features (key points + feature descriptors) from current frame
   * and 3d map points observed from the last frame projected onto the current
   * frame, (some are not valid, indicated by mask) we also know the descriptors
   * of each projected points
   *
   * Then for each projected map point, we need to find out the same point
   * observed in current frame, based on descriptor distance. return the matched
   * map points index and current key point index pairs
   *
   * Once we have the matched map points indexes for the newly observed points,
   * we can perform again a bundle adjustment.
   * */
  std::vector<std::pair<size_t, size_t>> matched_pairs;
  for (size_t i = 0; i < projected_map_points.size(); ++i) {
    const auto &current_key_points = features.GetUndistortedKeyPoints();
    const auto &current_points_descriptors = features.GetDescriptors();
    for (size_t j = 0; j < current_key_points.size(); ++j) {
      projected_map_points[i];
      map_points_descriptors[i];
      bool matched = false;
      if (matched) {
        matched_pairs.push_back({i, j});
      }
    }
  }
  return matched_pairs;
}

} // namespace clean_slam