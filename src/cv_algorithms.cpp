//
// Created by root on 4/18/20.
//
#include "cv_algorithms.h"
#include "cv_utils.h"
#include "match_map.h"
#include <boost/range/adaptor/indexed.hpp>
#include <cv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
namespace clean_slam {

g2o::SE3Quat GetVelocity(const g2o::SE3Quat &Tcw_current,
                         const g2o::SE3Quat &Tcw_prev) {
  return Tcw_current * Tcw_prev.inverse();
}

bool KeyPointWithinRadius(const cv::KeyPoint &key_point,
                          const Eigen::Vector2d &point, float radius) {
  const auto distance_sqr = std::pow((point.x() - key_point.pt.x), 2) +
                            std::pow((point.y() - key_point.pt.y), 2);
  return distance_sqr <= std::pow(radius, 2);
}

BoundF Calculate3DPointDistanceBound(const g2o::SE3Quat &Tcw,
                                     const cv::KeyPoint &key_point,
                                     const Eigen::Vector3d &point_3d,
                                     const OctaveScales &octave_scales) {
  const auto pose_in_world = Tcw.inverse();
  const auto distance = (point_3d - pose_in_world.translation()).norm();
  float octave_scale = octave_scales[key_point.octave];
  const float max_distance = octave_scale * distance;
  const float min_distance =
      max_distance / octave_scales[octave_scales.size() - 1];
  return {min_distance, max_distance};
}

Eigen::Vector3d ViewingDirection(const g2o::SE3Quat &Tcw,
                                 const Eigen::Vector3d &point_3d) {
  const auto pose_in_world = Tcw.inverse();
  Eigen::Vector3d dir = point_3d - pose_in_world.translation();
  dir.normalize();
  return dir;
}

std::vector<cv::DMatch> SearchByProjection(
    const OrbFeatureMatcher &matcher,
    const std::vector<Eigen::Vector2d> &projected_map_points,
    const cv::Mat &projected_map_points_descriptors,
    const std::vector<uint8_t> &map_points_octaves, const OrbFeatures &features,
    const cv::Mat &mask, int search_radius, const OctaveScales &octave_scales,
    int num_nearest_neighbor, int descriptor_distance_threshold) {

  /*
   * We have orb features (key points + feature descriptors) from current frame,
   * member and 3d map points observed from the last frame projected onto the
   * current frame, (some are not valid, indicated by mask) we also know the
   * descriptors of each projected points
   *
   * Then for each projected map point, we need to find out the same point
   * observed in current frame, based on descriptor distance. return the matched
   * map points index and current key point index pairs
   *
   * Once we have the matched map points indexes for the newly observed points,
   * we can perform again a bundle adjustment.
   * */
  const auto &current_descriptors = features.GetDescriptors();
  std::vector<std::vector<cv::DMatch>> matches_for_map_points =
      matcher.KnnMatch(projected_map_points_descriptors, current_descriptors,
                       num_nearest_neighbor);

  const auto &current_key_points = features.GetUndistortedKeyPoints();

  MatchMap map;

  for (const auto &matches_per_map_point :
       matches_for_map_points | boost::adaptors::indexed()) {

    if (!mask.empty() && !mask.at<uint8_t>(matches_per_map_point.index()))
      continue;

    for (const cv::DMatch &m : matches_per_map_point.value()) {
      const auto &projected_map_point = projected_map_points[m.queryIdx];
      const auto &current_key_point = current_key_points[m.trainIdx];

      // find the first one which satisfies the condition, then stop
      BoundI octave_bound{map_points_octaves[m.queryIdx] - 1,
                          map_points_octaves[m.queryIdx] + 1};
      if (octave_bound.IsWithIn(current_key_point.octave) &&
          KeyPointWithinRadius(current_key_point, projected_map_point,
                               search_radius *
                                   octave_scales[current_key_point.octave]) &&
          m.distance <= descriptor_distance_threshold) {

        map.Emplace(m);
        break;
      }
    }
  }
  return map.ToVector();
}

Eigen::Matrix3d GetEssentialMatrix(const g2o::SE3Quat &Tc0w,
                                   const g2o::SE3Quat &Tc1w) {
  const auto Tc0c1 = Tc0w * Tc1w.inverse();
  return g2o::skew(Tc0c1.translation()) * Tc0c1.rotation();
}

Eigen::Matrix3d GetFundamentalMatrix(const g2o::SE3Quat &Tc0w,
                                     const g2o::SE3Quat &Tc1w,
                                     const Eigen::Matrix3d &intrinsics) {
  // p0*F*p1 = 0
  auto E = GetEssentialMatrix(Tc0w, Tc1w);
  return intrinsics.transpose().inverse() * E * intrinsics.inverse();
}

Eigen::Matrix<double, 3, 4>
GetProjectionMatrix(const g2o::SE3Quat &Tcw,
                    const Eigen::Matrix3d &intrinsics) {
  Eigen::Matrix<double, 3, 4> projection_matrix;
  projection_matrix.block<3, 3>(0, 0) =
      intrinsics * Tcw.rotation().toRotationMatrix();
  projection_matrix.block<3, 1>(0, 3) = intrinsics * Tcw.translation();
  return projection_matrix;
}

double DistanceToEpipolarLine(const cv::Point2f &point0,
                              const Eigen::Matrix3d &fundamental_mat,
                              const cv::Point2f &point1) {
  Eigen::Vector3d point0_homo{point0.x, point0.y, 1};
  Eigen::Vector3d point1_homo{point1.x, point1.y, 1};
  auto epipolar_line = fundamental_mat * point1_homo;
  auto distance =
      std::abs(epipolar_line.dot(point0_homo)) /
      std::sqrt(std::pow(epipolar_line[0], 2) + std::pow(epipolar_line[1], 2));
  return distance;
}

cv::Mat TriangulatePoints(const g2o::SE3Quat &Tc0w,
                          const std::vector<cv::Point2f> &points0,
                          const g2o::SE3Quat &Tc1w,
                          const std::vector<cv::Point2f> &points1,
                          const Eigen::Matrix3d &K) {
  cv::Mat points_3d_cartisian;
  if (points0.empty())
    return points_3d_cartisian;
  cv::Mat points_3d_homo;
  cv::Mat projection_mat0, projection_mat1;
  cv::eigen2cv(GetProjectionMatrix(Tc0w, K), projection_mat0);
  cv::eigen2cv(GetProjectionMatrix(Tc1w, K), projection_mat1);

  cv::triangulatePoints(projection_mat0, projection_mat1, points0, points1,
                        points_3d_homo);
  cv::convertPointsFromHomogeneous(points_3d_homo.t(), points_3d_cartisian);
  // points in row wise
  return points_3d_cartisian;
}

void ValidateMapPoints(const cv::Mat &triangulated_points,
                       const g2o::SE3Quat &Tcw, cv::Mat &mask) {
  if (mask.empty()) {
    mask = cv::Mat(triangulated_points.rows, 1, CV_8U, 1U);
  }
  for (size_t i{0}; i < triangulated_points.rows; ++i) {
    const auto point_3d = ToVector3d<float>(triangulated_points.row(i));
    const auto point_in_cam_frame = Tcw.map(point_3d);
    // positive depth
    mask.row(i) = mask.row(i) & (point_in_cam_frame[2] > 0);
  }
}
} // namespace clean_slam