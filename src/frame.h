//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_H_
#define CLEAN_SLAM_SRC_FRAME_H_
#include "map.h"
#include "orb_extractor.h"
#include "views/elements_view.h"
#include <opencv2/core/core.hpp>

namespace clean_slam {

using Points3DView = ElementsView<std::vector<Eigen::Vector3d>>;
using DescriptorsView = ElementsView<cv::Mat, cv::Matx<uint8_t, 1, 32>>;
class Frame {
public:
  Frame() = default;
  Frame(std::vector<cv::KeyPoint> &&key_points,
        std::vector<size_t> &&map_point_indexes, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp)
      : _key_points{std::move(key_points)},
        _map_point_indexes(std::move(map_point_indexes)), _map{map}, _Tcw{Tcw},
        _timestamp{timestamp} {
    assert(_key_points.size() == _map_point_indexes.size());
  }

  Frame(const std::vector<cv::KeyPoint> &key_points,
        const std::vector<size_t> &map_point_indexes, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp)
      : _key_points{key_points},
        _map_point_indexes(map_point_indexes), _map{map}, _Tcw{Tcw},
        _timestamp{timestamp} {
    assert(_key_points.size() == _map_point_indexes.size());
  }

  const g2o::SE3Quat &GetTcw() const { return _Tcw; }
  double GetTimestamp() const { return _timestamp; }

  const std::vector<cv::KeyPoint> &GetKeyPoints() const;
  Points3DView GetPoints3DView() const;
  DescriptorsView GetDescriptorsView() const;

private:
  std::vector<cv::KeyPoint> _key_points;
  std::vector<size_t> _map_point_indexes;
  const Map *_map;
  g2o::SE3Quat _Tcw;
  double _timestamp;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
