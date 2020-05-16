//
// Created by root on 5/16/20.
//
#include "iterator.h"
namespace clean_slam {

template <>
const DescriptorT *Iterator<cv::Mat, DescriptorT>::operator->() const {
  const auto pos = _indexes[_pos];
  return _elements.ptr<DescriptorT>(pos);
}

} // namespace clean_slam