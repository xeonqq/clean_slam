//
// Created by root on 5/16/20.
//
#include "elements_view.h"

namespace clean_slam {

template <>
const DescriptorT &
    ElementsView<cv::Mat, DescriptorT>::operator[](size_t pos) const {
  return *_elements.ptr<DescriptorT>(_indexes[pos]);
};

template <> size_t ElementsView<cv::Mat, DescriptorT>::size() const {
  return _elements.rows;
}
} // namespace clean_slam
