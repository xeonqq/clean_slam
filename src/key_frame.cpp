#include "key_frame.h"
#include "frame.h"
#include "orb_feature_matcher.h"
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>
namespace clean_slam {

KeyFrame::KeyFrame(Frame &frame, vertex_t vertex)
    : _frame(&frame), _vertex(vertex) {
  // todo: set ref kf based on most shared map points
  _frame->SetRefKf(vertex);
  for (auto map_point : _frame->GetMatchedMapPointsRng()) {
    _connections.push_back(map_point->AddObserver([&]() { return this; }));
  }
}

const g2o::SE3Quat &KeyFrame::GetTcw() const { return _frame->GetTcw(); }

const std::vector<cv::KeyPoint> &KeyFrame::GetUndistortedKeyPoints() const {
  return _frame->GetUndistortedKeyPoints();
}
const cv::Mat &KeyFrame::GetDescriptors() const {
  return _frame->GetDescriptors();
}

boost::select_first_range<std::map<MapPoint *, size_t>>
KeyFrame::GetMatchedMapPointsRng() const {
  return _frame->GetMatchedMapPointsRng();
}

TriangulationResult
KeyFrame::MatchUnmatchedKeyPoints(const OrbFeatureMatcher &matcher,
                                  KeyFrame &key_frame,
                                  const cv::Mat &camera_instrinsics) {
  return _frame->MatchUnmatchedKeyPoints(matcher, *key_frame._frame,
                                         camera_instrinsics);
}

void KeyFrame::AddMatchedMapPoint(MapPoint *map_point, size_t index) {
  _connections.push_back(map_point->AddObserver([&]() { return this; }));
  _frame->_matched_map_point_to_idx.emplace(map_point, index);
}

void KeyFrame::EraseMapPoint(MapPoint *map_point) {
  _frame->_matched_map_point_to_idx.erase(map_point);
}

size_t KeyFrame::GetNumKeyPoints() const { return _frame->GetNumKeyPoints(); }

size_t KeyFrame::GetNumMatchedMapPoints() const {
  return _frame->GetNumMatchedMapPoints();
}

cv::Mat KeyFrame::GetMatchedKeyPointDescriptor(MapPoint *map_point) const {
  return _frame->GetMatchedKeyPointDescriptor(map_point);
}

KeyFrame::~KeyFrame() {
  std::for_each(_connections.begin(), _connections.end(),
                [](auto &c) { c.disconnect(); });
}

vertex_t KeyFrame::GetVertex() const { return _vertex; }

} // namespace clean_slam
