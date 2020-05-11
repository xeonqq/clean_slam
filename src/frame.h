//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_H_
#define CLEAN_SLAM_SRC_FRAME_H_
#include "map.h"
#include "orb_extractor.h"
#include <opencv2/core/core.hpp>

namespace clean_slam {
class Frame {
public:
  Frame() = default;
  Frame(std::vector<size_t> &&map_point_indexes, const Map *map,
        const g2o::SE3Quat &Tcw)
      : _map_point_indexes(std::move(map_point_indexes)), _map{map}, _Tcw{Tcw} {
  }

  const g2o::SE3Quat &GetTcw() const { return _Tcw; }

private:
  std::vector<size_t> _map_point_indexes;
  const Map *_map;
  g2o::SE3Quat _Tcw;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
