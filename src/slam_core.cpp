//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include <boost/foreach.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/combine.hpp>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <third_party/spdlog/spdlog.h>
#define DEBUG
namespace clean_slam {

const double kViewAngleCosThreshold = std::cos(M_PI / 3);

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
  // const velocity model
  const auto &prev_frame = _frames.back();
  const auto &prev_prev_frame = _frames[_frames.size() - 2];
  const auto &prev_Tcw = prev_frame.GetTcw();
  g2o::SE3Quat velocity =
      clean_slam::GetVelocity(prev_Tcw, prev_prev_frame.GetTcw());
  auto Tcw = velocity * prev_Tcw;

  Frame frame{_orb_extractor->DetectAndUndistortKeyPoints(image), &_map, Tcw,
              timestamp, prev_frame.GetRefKfVertex()};

  // project the matched map points in the previous frame to current frame
  std::vector<Eigen::Vector2d> points_reprojected = frame.ReprojectPoints3d(
      GetMapPointsPositions(prev_frame.GetMatchedMapPoints()),
      _camera_intrinsic);

  cv::Mat mask = cv::Mat(points_reprojected.size(), 1, CV_8U);

  const auto &x_bounds = _undistorted_image_boundary.GetXBounds();
  const auto &y_bounds = _undistorted_image_boundary.GetYBounds();

  for (size_t i = 0; i < points_reprojected.size(); ++i) {
    mask.at<uint8_t>(i) =
        IsPointWithInBounds(points_reprojected[i], x_bounds, y_bounds);
  }
  const int search_radius = 7;
  auto matches =
      frame.SearchByProjection(_orb_feature_matcher, points_reprojected,
                               prev_frame, mask, search_radius, _octave_scales);
  if (matches.size() < 20) {
    spdlog::info(
        "Search again with larger radius, num matched map points < 20: {}",
        matches.size());
    matches = frame.SearchByProjection(_orb_feature_matcher, points_reprojected,
                                       prev_frame, mask, search_radius * 2,
                                       _octave_scales);
    spdlog::info("After search again , num matched map points: {}",
                 matches.size());
  }

  TrackLocalMap(frame);
  const int kNumMatchedMapPointsForBA = 20;
  if (matches.size() > kNumMatchedMapPointsForBA) {
    spdlog::info("Num matched map points: {}", matches.size());
    frame.OptimizePose(_optimizer_only_pose);
  } else {
    spdlog::warn("Num matched map points < {}, only: {}",
                 kNumMatchedMapPointsForBA, matches.size());
    return;
  }

  _frames.push_back(std::move(frame));
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

  const auto &current_frame = _frames.back();
  if (_viewer) {
    _viewer->OnNotify(Content{Tcw, {}});
    _viewer->OnNotify(image, current_frame.GetOrbFeatures());
  }

  if ((matches.size() < _frames.back().GetRefKeyFrameNumKeyPoints() * 0.9) &&
      current_frame.GetOrbFeatures().NumKeyPoints() > 50) {
  }
}

void SlamCore::TrackLocalMap(Frame &current_frame) {
  auto key_frames_to_track_local_map =
      current_frame.GetKeyFramesToTrackLocalMap();
  std::set<const MapPoint *> map_points_to_project;
  for (auto kf_vertex : key_frames_to_track_local_map) {
    const auto &key_frame = _map.GetKeyFrame(kf_vertex);
    boost::copy(
        key_frame.GetMatchedMapPoints(),
        std::inserter(map_points_to_project, map_points_to_project.end()));
  }
  for (auto map_point : current_frame.GetMatchedMapPoints()) {
    map_points_to_project.erase(map_point);
  }

  std::vector<Eigen::Vector2d> points_reprojected =
      current_frame.ReprojectPoints3d(
          GetMapPointsPositions(map_points_to_project), _camera_intrinsic);

  const auto &x_bounds = _undistorted_image_boundary.GetXBounds();
  const auto &y_bounds = _undistorted_image_boundary.GetYBounds();

  std::vector<Eigen::Vector2d> valid_map_points_reprojected;
  cv::Mat valid_map_points_descriptors;
  std::vector<uint8_t> valid_map_points_octaves;
  for (const auto &zipped :
       boost::combine(points_reprojected, map_points_to_project) |
           boost::adaptors::indexed()) {
    Eigen::Vector2d point_reprojected;
    const MapPoint *map_point;
    boost::tie(point_reprojected, map_point) = zipped.value();

    //  1. check points in undistorted range
    if (!IsPointWithInBounds(point_reprojected, x_bounds, y_bounds))
      continue;

    // 2. check viewing direction feasible
    auto camera_pose_in_world = current_frame.GetTcw().inverse();
    const auto viewing_direction =
        (map_point->GetPoint3D() - camera_pose_in_world.translation())
            .normalized();
    const auto proj = map_point->GetViewDirection().dot(viewing_direction);
    if (proj < kViewAngleCosThreshold)
      continue;

    //  3. check new depth of points (wrt to new camera pose) is within
    //  scale pyramid range
    const auto depth =
        (map_point->GetPoint3D() - camera_pose_in_world.translation()).norm();
    if (!map_point->IsObservableFromDistance(depth))
      continue;

    int predicted_octave_level =
        map_point->PredictOctaveScale(depth, _octave_scales);
    valid_map_points_octaves.push_back(predicted_octave_level);
    valid_map_points_reprojected.push_back(point_reprojected);
    valid_map_points_descriptors.push_back(
        map_point->GetRepresentativeDescriptor());
  }
  auto matches = current_frame.SearchUnmatchedKeyPointsByProjection(
      _orb_feature_matcher, valid_map_points_reprojected,
      valid_map_points_descriptors, valid_map_points_octaves, 7,
      _octave_scales);
  spdlog::info("Num additional matched map points track local map: {}",
               matches.size());
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
