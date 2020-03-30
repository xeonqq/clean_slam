//
// Created by root on 3/23/20.
//

#include "initializer.h"
#include "cv_utils.h"
#include <opencv2/core/eigen.hpp>
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {
Initializer::Initializer(const cv::Mat &camera_instrinsics)
    : _camera_motion_estimator{camera_instrinsics}, _bundle_adjustment{
                                                        camera_instrinsics} {}

bool Initializer::Initialize(
    const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame) {

  PlausibleTransformation plausible_transformation =
      _camera_motion_estimator.Estimate(points_previous_frame,
                                        points_current_frame);
  //    if (true) {
  if (plausible_transformation.IsGood()) {
    _plausible_transformation = plausible_transformation;
    // todo: bundle adjustment

    return true;
  } else
    return false;
}

void Initializer::RunBundleAdjustment(
    const KeyPointsPair &matched_key_points_pair) {

  _bundle_adjustment.AddPose(0, g2o::SE3Quat{}, true);
  _bundle_adjustment.AddPose(1,
                             _plausible_transformation.GetHomogeneousMatrix());

  const int kPoint3DInitialId = 2;
  const cv::Mat good_triangulated_points =
      _plausible_transformation.GetGoodTriangulatedPoints();
  for (int i = 0; i < good_triangulated_points.rows; ++i) {
    const auto point_3d = good_triangulated_points.row(i);
    _bundle_adjustment.AddPoint3D(kPoint3DInitialId + i,
                                  ToVector3d<float>(point_3d));
  }
  const auto good_key_points_prev_frame =
      FilterByMask(matched_key_points_pair.first,
                   _plausible_transformation.GetGoodPointsMask());
  const auto good_key_points_curr_frame =
      FilterByMask(matched_key_points_pair.second,
                   _plausible_transformation.GetGoodPointsMask());

  std::array<const std::vector<cv::KeyPoint> *, 2> pose_id_to_points = {
      &good_key_points_prev_frame, &good_key_points_curr_frame};
  for (size_t pose_id = 0; pose_id < pose_id_to_points.size(); ++pose_id) {
    for (size_t point_id = 0; point_id < pose_id_to_points[pose_id]->size();
         ++point_id) {
      const auto key_points = *pose_id_to_points[pose_id];
      _bundle_adjustment.AddEdge(
          kPoint3DInitialId + point_id, pose_id,
          Point2fToVector2d(key_points[point_id].pt),
          Eigen::Matrix2d::Identity() *
              _octave_sigma_scales[key_points[point_id].octave]);
    }
  }
  _bundle_adjustment.Optimize(20, true);
  //  spdlog::info("pose after bundle adjustment: {}", );
  std::cerr << _bundle_adjustment.GetOptimizedPose() << std::endl;
}

HomogeneousMatrix Initializer::GetHomogeneousMatrix() const {
  return _plausible_transformation.GetHomogeneousMatrix();
}
cv::Mat Initializer::GetTriangulatedPoints() const {
  return _plausible_transformation.GetTriangulatedPoints();
}
cv::Mat Initializer::GetGoodTriangulatedPoints() const {
  return _plausible_transformation.GetGoodTriangulatedPoints();
}

} // namespace clean_slam
