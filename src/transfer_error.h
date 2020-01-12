//
// Created by root on 1/11/20.
//

#ifndef CLEAN_SLAM_SRC_TRANSFER_ERROR_H_
#define CLEAN_SLAM_SRC_TRANSFER_ERROR_H_
#include <opencv2/core/core.hpp>
#include <vector>

namespace clean_slam {
class Homography {
public:
  static float
  CalculateSymmetricTransferError(const std::vector<cv::Point2f> &src_points,
                                  const std::vector<cv::Point2f> &dst_points,
                                  const cv::Mat &H, const cv::Mat &inlies_mask);
  static float
  CalculateTransferError(const std::vector<cv::Point2f> &src_points,
                         const std::vector<cv::Point2f> &dst_points,
                         const cv::Mat &m, const cv::Mat &inlies_mask);
};

class EpipolarConstraint {
public:
  static float
  CalculateSymmetricTransferError(const std::vector<cv::Point2f> &src_points,
                                  const std::vector<cv::Point2f> &dst_points,
                                  const cv::Mat &F, const cv::Mat &inlies_mask);
  static cv::Mat
  CalculateEpipolarLine(const std::vector<cv::Point2f> &src_points,
                        const cv::Mat &F);
  static cv::MatExpr
  CalculateRepojectionErrorDemoniator(const cv::Mat &epipolar_lines);
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_TRANSFER_ERROR_H_
