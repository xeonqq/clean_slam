#include "key_frame.h"
namespace clean_slam {

void KeyFrame::AddMatchedMapPoint(MapPoint *map_point) {
  _connections.push_back(map_point->AddObserver(
      [&](MapPoint *map_point) { _matched_map_points.erase(map_point); }));
  _matched_map_points.insert(map_point);
}

size_t KeyFrame::NumberOfMatchedMapPoints() const {
  return _matched_map_points.size();
}

KeyFrame::~KeyFrame() {
  std::for_each(_connections.begin(), _connections.end(),
                [](auto &c) { c.disconnect(); });
}
} // namespace clean_slam
