//
// Created by root on 1/3/20.
//

#ifndef CLEAN_SLAM_SRC_FRAME_H_
#define CLEAN_SLAM_SRC_FRAME_H_
#include "map.h"
#include "orb_extractor.h"
#include <iterator>
#include <opencv2/core/core.hpp>

namespace clean_slam {
template <typename T> class Iterator {
public:
  using value_type = T;
  using pointer = const value_type *;
  using reference = const value_type &;
  using difference_type = size_t;
  using iterator_category = std::forward_iterator_tag;
  Iterator(const std::vector<T> &elements, const std::vector<size_t> &indexes,
           size_t pos)
      : _elements(elements), _indexes(indexes), _pos{pos} {}

  Iterator(const Iterator &) = default;

  Iterator &operator++() {
    ++_pos;
    return *this;
  }

  Iterator operator++(int) {
    Iterator origin = *this;
    ++(*this);
    return origin;
  };

  pointer operator->() const {
    const auto pos = _indexes[_pos];
    return &_elements[pos];
  }

  reference operator*() const { return *operator->(); }

  bool operator==(const Iterator &other) const { return other._pos == _pos; }

  inline bool operator!=(const Iterator &other) const {
    return !(*this == other);
  }

  bool operator<(const Iterator &other) const { return _pos < other._pos; }

  bool operator>(const Iterator &other) const { return _pos > other._pos; }

  inline bool operator<=(const Iterator &other) const {
    return !(*this > other);
  }

  inline bool operator>=(const Iterator &other) const {
    return !(*this < other);
  }

private:
  const std::vector<T> &_elements;
  const std::vector<size_t> &_indexes;
  size_t _pos;
};

class Points3DView {
  const std::vector<Eigen::Vector3d> &_points_3d;
  const std::vector<size_t> &_indexes;

public:
  using iterator = Iterator<Eigen::Vector3d>;
  using const_iterator = Iterator<Eigen::Vector3d>;
  Points3DView(const std::vector<Eigen::Vector3d> &points_3d,
               const std::vector<size_t> &indexes)
      : _points_3d(points_3d), _indexes(indexes) {}

  iterator begin() const;
  iterator end() const;

  const Eigen::Vector3d &operator[](size_t pos) const {
    return _points_3d[_indexes[pos]];
  };
  size_t size() const { return _indexes.size(); }
};

class Frame {
public:
  Frame() = default;
  Frame(std::vector<size_t> &&map_point_indexes, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp)
      : _map_point_indexes(std::move(map_point_indexes)), _map{map}, _Tcw{Tcw},
        _timestamp{timestamp} {}

  Frame(std::vector<size_t> &map_point_indexes, const Map *map,
        const g2o::SE3Quat &Tcw, double timestamp)
      : _map_point_indexes(map_point_indexes), _map{map}, _Tcw{Tcw},
        _timestamp{timestamp} {}

  const g2o::SE3Quat &GetTcw() const { return _Tcw; }
  double GetTimestamp() const { return _timestamp; }

  Points3DView GetPoints3DView() const;

private:
  std::vector<size_t> _map_point_indexes;
  const Map *_map;
  g2o::SE3Quat _Tcw;
  double _timestamp;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SRC_FRAME_H_
