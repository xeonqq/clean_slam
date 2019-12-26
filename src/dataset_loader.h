//
// Created by root on 12/26/19.
//

#ifndef CLEAN_SLAM_SRC_DATASET_LOADER_H_
#define CLEAN_SLAM_SRC_DATASET_LOADER_H_

#include "third_party/g2o/g2o/types/se3quat.h"
#include <string>
#include <vector>

namespace clean_slam {

bool IsCommentLine(const std::string &s);

struct ImageFile {
  ImageFile(const std::string &image_filename, double timestamp)
      : image_filename(image_filename), timestamp(timestamp) {}
  std::string image_filename;
  double timestamp;
};

class GroundTruth {
public:
  GroundTruth(const Eigen::Vector3d &translation,
              const Eigen::Quaterniond &quaternion, double timestamp)
      : _se3_quat{quaternion, translation}, _timestamp{timestamp} {}

private:
  g2o::SE3Quat _se3_quat;
  double _timestamp;
};

class DatasetLoader {
public:
  void LoadFreiburgRgb(const std::string &dataset_folder_name);
  const std::vector<ImageFile> &GetImageFiles() const;

private:
  std::vector<ImageFile> _image_files;
  std::vector<GroundTruth> _ground_truths;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_DATASET_LOADER_H_
