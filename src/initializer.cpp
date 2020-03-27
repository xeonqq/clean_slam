//
// Created by root on 3/23/20.
//

#include "initializer.h"
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
