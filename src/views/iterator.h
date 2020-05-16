//
// Created by root on 5/16/20.
//

#ifndef CLEAN_SLAM_SRC_VIEWS_ITERATOR_H_
#define CLEAN_SLAM_SRC_VIEWS_ITERATOR_H_

#include "descriptor.h"
#include <iterator>
#include <opencv2/core/mat.hpp>
#include <vector>

namespace clean_slam {
template <typename ElemContainer,
          typename T = typename ElemContainer::value_type>
class Iterator {
public:
  using value_type = T;
  using pointer = const value_type *;
  using reference = const value_type &;
  using difference_type = size_t;
  using iterator_category = std::forward_iterator_tag;

  Iterator(const ElemContainer &elements, const std::vector<size_t> &indexes,
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
  const ElemContainer &_elements;
  const std::vector<size_t> &_indexes;
  size_t _pos;
};

template <>
const DescriptorT *Iterator<cv::Mat, DescriptorT>::operator->() const;
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_VIEWS_ITERATOR_H_
