//
// Created by root on 4/18/20.
//
#include "cv_algorithms.h"
#include <boost/range/adaptor/indexed.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
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

std::vector<cv::DMatch>
SearchByProjection(const OrbFeatures &features,
                   const std::vector<Eigen::Vector2d> &projected_map_points,
                   const std::vector<uint8_t> &map_points_octaves,
                   const cv::Mat &map_points_descriptors, const cv::Mat &mask,
                   int search_radius) {
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
  auto matcher =
      cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
  std::vector<std::vector<cv::DMatch>> matches_for_map_points;
  const auto &current_descriptors = features.GetDescriptors();

  // for flann matcher mask is not supported
  matcher.knnMatch(map_points_descriptors, current_descriptors,
                   matches_for_map_points,
                   std::min(current_descriptors.rows, 5));

  const auto &current_key_points = features.GetUndistortedKeyPoints();

  std::vector<cv::DMatch> matched_pairs;
  matched_pairs.reserve(map_points_octaves.size());
  const int descriptor_distance_threshold = 10;

  for (const auto &matches_per_map_point :
       matches_for_map_points | boost::adaptors::indexed()) {

    if (!mask.at<uint8_t>(matches_per_map_point.index()))
      continue;

    for (const cv::DMatch &m : matches_per_map_point.value()) {
      const auto &projected_map_point = projected_map_points[m.queryIdx];
      const auto &current_key_point = current_key_points[m.trainIdx];

      // find the first one which satisfies the condition, then stop
      if (current_key_point.octave == map_points_octaves[m.queryIdx] &&
          KeyPointWithinRadius(current_key_point, projected_map_point,
                               search_radius) &&
          m.distance <= descriptor_distance_threshold) {
        matched_pairs.push_back(m);
        break;
      }
    }
  }
  return matched_pairs;
}

bool KeyPointWithinRadius(const cv::KeyPoint &key_point,
                          const Eigen::Vector2d &point, float radius) {
  const auto distance_sqr = std::pow((point.x() - key_point.pt.x), 2) +
                            std::pow((point.y() - key_point.pt.y), 2);
  return distance_sqr <= std::pow(radius, 2);
}
} // namespace clean_slam