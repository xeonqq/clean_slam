//
// Created by root on 5/16/20.
//

#ifndef CLEAN_SLAM_SRC_VIEWS_DESCRIPTOR_H_
#define CLEAN_SLAM_SRC_VIEWS_DESCRIPTOR_H_

#include <opencv2/core/matx.hpp>
namespace clean_slam {
using DescriptorT = cv::Matx<uint8_t, 1, 32>;
}
#endif // CLEAN_SLAM_SRC_VIEWS_DESCRIPTOR_H_
