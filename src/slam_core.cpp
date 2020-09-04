//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include <boost/foreach.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <third_party/spdlog/spdlog.h>

#define DEBUG
namespace clean_slam {

SlamCore::SlamCore(const cv::Mat &camera_intrinsics,
                   const cv::Mat &camera_distortion_coeffs,
                   OrbExtractor *orb_extractor, Optimizer *optimizer,
                   OptimizerOnlyPose *optimizer_only_pose, Viewer *viewer,
                   const OctaveScales &octave_scale)
    : _camera_intrinsic(camera_intrinsics), _orb_extractor(orb_extractor),
      _optimizer{optimizer}, _optimizer_only_pose{optimizer_only_pose},
      _viewer(viewer), _undistorted_image_boundary{camera_intrinsics,
                                                   camera_distortion_coeffs},
      _octave_scales{octave_scale}, _map{_octave_scales} {}

void SlamCore::ProcessFirstImage(const cv::Mat &image, double timestamp) {
  _undistorted_image_boundary.ComputeUndistortedCorners(image);
}

void SlamCore::TrackByMotionModel(const cv::Mat &image, double timestamp) {
  const auto orb_features = _orb_extractor->DetectAndUndistortKeyPoints(image);
  // const velocity model
  const auto &prev_frame = _frames.back();
  const auto &prev_prev_frame = _frames[_frames.size() - 2];
  const auto &prev_Tcw = prev_frame.GetTcw();
  g2o::SE3Quat velocity =
      clean_slam::GetVelocity(prev_Tcw, prev_prev_frame.GetTcw());
  auto Tcw = velocity * prev_Tcw;
  //   const auto camera_pose_in_world = Tcw.inverse();

  // project the map points in the reference key frame to current frame, given
  // the current frame's pose
  std::vector<Eigen::Vector2d> points_reprojected =
      prev_frame.ReprojectPoints3d(Tcw, _camera_intrinsic);
  cv::Mat mask = cv::Mat(points_reprojected.size(), 1, CV_8U);

  const auto &x_bounds = _undistorted_image_boundary.GetXBounds();
  const auto &y_bounds = _undistorted_image_boundary.GetYBounds();

  for (size_t i = 0; i < points_reprojected.size(); ++i) {
    mask.at<uint8_t>(i) =
        IsPointWithInBounds(points_reprojected[i], x_bounds, y_bounds);
  }
  const int search_radius = 7;
  auto matches = prev_frame.SearchByProjection(
      _orb_feature_matcher, orb_features, points_reprojected, mask,
      search_radius, _octave_scales);
  if (matches.size() < 20) {
    spdlog::info(
        "Search again with larger radius, num matched map points < 20: {}",
        matches.size());
    matches = prev_frame.SearchByProjection(_orb_feature_matcher, orb_features,
                                            points_reprojected, mask,
                                            search_radius * 2, _octave_scales);
    spdlog::info("After search again , num matched map points: {}",
                 matches.size());
  }

  auto matched_key_points_current_frame = FilterByIndex(
      orb_features.GetUndistortedKeyPoints(),
      matches | boost::adaptors::transformed(
                    [](const auto &match) { return match.trainIdx; }));

  const auto matched_map_points = prev_frame.GetMatchedMapPoints(matches);

  const int kNumMatchedMapPointsForBA = 20;
  if (matched_map_points.size() > kNumMatchedMapPointsForBA) {
    spdlog::info("Num matched map points: {}",
                 matched_key_points_current_frame.size());
    _optimizer_only_pose->Clear();
    Tcw = _optimizer_only_pose->Optimize(
        Tcw, matched_key_points_current_frame,
        GetMapPointsPositions(matched_map_points));
  } else {
    spdlog::warn("Num matched map points < {}, only: {}",
                 kNumMatchedMapPointsForBA, matched_map_points.size());
    return;
  }

  _frames.emplace_back(std::move(matched_key_points_current_frame),
                       std::move(matched_map_points), &_map, Tcw, timestamp,
                       prev_frame.GetRefKfVertex());
#if 0
  static int i = 0;
  ++i;
  cv::Mat out;
  try {
    cv::drawMatches(_key_frame_graph.GetReferenceKeyFrameImage(),
                    prev_frame.GetKeyPoints(), image,
                    orb_features.GetUndistortedKeyPoints(), matches, out);
  } catch (std::exception &e) {
    std::cerr << "TrackException: " << e.what() << std::endl;
  }
  std::string png_name = std::string("matches_by_motion_track_knn") +
                         std::to_string(i) + std::string(".png");
  cv::imwrite(png_name, out);
#endif

  /*
    for (size_t i = 0; i < points_reprojected.size(); ++i) {
      const auto &point = points_reprojected[i];

      //  1. check points in undistorted range
      if (!IsPointWithInBounds(point, x_bounds, y_bounds))
        continue;

      //  2. check new depth of points (wrt to new camera pose) is within
      //  scale pyramid range
          const auto depth =
              (points_3d[i] - camera_pose_in_world.translation()).norm();
          if (!distance_bounds[i].IsWithIn(depth))
            continue;

          int predicted_octave_level =
              _octave_scales.MapDistanceToOctaveLevel(depth,
              distance_bounds[i]);

      mask[i] = true;
    }
    */

  if (_viewer) {
    _viewer->OnNotify(Content{Tcw, {}});
    _viewer->OnNotify(image, orb_features);
  }

  if ((matches.size() < _frames.back().GetRefKeyFrameNumKeyPoints() * 0.9) &&
      orb_features.NumKeyPoints() > 50) {
  }
}

void SlamCore::TrackLocalMap() {
  auto &current_frame = _frames.back();
  auto key_frames_for_local_mapping =
      current_frame.GetKeyFramesForLocalMapping();
  std::set<const MapPoint *> map_points_to_project;
  for (auto kf_vertex : key_frames_for_local_mapping) {
    const auto &key_frame = _map.GetKeyFrame(kf_vertex);
    boost::copy(
        key_frame.GetMatchedMapPoints(),
        std::inserter(map_points_to_project, map_points_to_project.end()));
  }
  // todo: filter already matched map points from current frame

  std::vector<Eigen::Vector2d> points_reprojected;
  points_reprojected.reserve(map_points_to_project.size());
  clean_slam::ReprojectPoints3d(GetMapPointsPositions(map_points_to_project),
                                std::begin(points_reprojected),
                                current_frame.GetTcw(), _camera_intrinsic);
}
CameraTrajectory SlamCore::GetTrajectory() const {
  CameraTrajectory trajectory;
  boost::range::transform(
      _frames, std::back_inserter(trajectory), [](const auto &frame) {
        return StampedTransformation{frame.GetTcw(), frame.GetTimestamp()};
      });
  return trajectory;
}

} // namespace clean_slam
