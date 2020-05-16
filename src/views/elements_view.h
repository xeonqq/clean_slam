//
// Created by root on 5/16/20.
//

#ifndef CLEAN_SLAM_SRC_VIEWS_ELEMENTS_VIEW_H_
#define CLEAN_SLAM_SRC_VIEWS_ELEMENTS_VIEW_H_

#include "descriptor.h"
#include "iterator.h"
#include <opencv2/core/mat.hpp>
#include <type_traits>
#include <vector>

namespace clean_slam {

template <typename ElemContainer,
          typename T = typename ElemContainer::value_type>
class ElementsView {
  const ElemContainer &_elements;
  const std::vector<size_t> &_indexes;

public:
  using iterator = Iterator<ElemContainer, T>;
  using const_iterator = Iterator<ElemContainer, T>;

  ElementsView(const ElemContainer &points_3d,
               const std::vector<size_t> &indexes)
      : _elements(points_3d), _indexes(indexes) {}

  iterator begin() const { return iterator{_elements, _indexes, 0}; }

  iterator end() const {
    return iterator{_elements, _indexes, _indexes.size()};
  }

  const T &operator[](size_t pos) const { return _elements[_indexes[pos]]; };

  size_t size() const { return _indexes.size(); }
};

template <>
const DescriptorT &
    ElementsView<cv::Mat, DescriptorT>::operator[](size_t pos) const;

template <> size_t ElementsView<cv::Mat, DescriptorT>::size() const;
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_VIEWS_ELEMENTS_VIEW_H_
