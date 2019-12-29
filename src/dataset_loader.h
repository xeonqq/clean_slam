//
// Created by root on 12/26/19.
//

#ifndef CLEAN_SLAM_SRC_DATASET_LOADER_H_
#define CLEAN_SLAM_SRC_DATASET_LOADER_H_

#include "ground_truth.h"
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>
namespace Eigen {
using Vector5d = Matrix<double, 5, 1>;
}

namespace clean_slam {

bool IsCommentLine(const std::string &s);

struct ImageFile {
  ImageFile(const std::string &image_filename, double timestamp)
      : image_filename(image_filename), timestamp(timestamp) {}
  std::string image_filename;
  double timestamp;
};

class DatasetLoader {
public:
  void LoadFreiburgDataset(const std::string &dataset_folder_name,
                           const std::string &path_to_yaml);
  const std::vector<ImageFile> &GetImageFiles() const;
  const std::vector<GroundTruth> &GetGroundTruths() const;
  GroundTruth GetGroundTruthAt(double timestamp) const;
  const std::string &GetDatasetFolder() const;
  const Eigen::Matrix3d &GetCameraIntrinsics() const;
  const Eigen::Matrix<double, 5, 1> &GetDistortionCoeffs() const;

private:
  void LoadFreiburgRgb(const std::string &dataset_folder_name);
  void LoadFreiburgGroundTruth(const std::string &dataset_folder_name);
  void LoadFreiburgCameraIntrinsics(const std::string &path_to_yaml);
  std::vector<ImageFile> _image_files;
  GroundTruths _ground_truths;
  std::string _dataset_folder;
  Eigen::Matrix3d _camera_intrinsics;
  Eigen::Matrix<double, 5, 1> _distortion_coeffs;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_DATASET_LOADER_H_
