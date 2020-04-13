//
// Created by root on 12/26/19.
//

#ifndef CLEAN_SLAM_SRC_DATASET_LOADER_H_
#define CLEAN_SLAM_SRC_DATASET_LOADER_H_

#include "ground_truth.h"
#include "settings.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
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

class DatasetLoader {
public:
  void LoadFreiburgDataset(const std::string &dataset_folder_name,
                           const std::string &path_to_yaml);
  void LoadImages(const std::string& image_folder, const std::string &path_to_yaml);
  const std::vector<ImageFile> &GetImageFiles() const;
  const std::vector<GroundTruth> &GetGroundTruths() const;
  GroundTruth GetGroundTruthAt(double timestamp) const;
  const std::string &GetDatasetFolder() const;
  const cv::Mat &GetCameraIntrinsics() const;
  const cv::Mat &GetDistortionCoeffs() const;
  const ViewerSettings &GetViewerSettings() const;
  const OrbExtractorSettings &GetOrbExtractorSettings() const;

private:
  void LoadRgb(const std::string &dataset_folder_name);
  void LoadFreiburgGroundTruth(const std::string &dataset_folder_name);
  void LoadCameraIntrinsics(const cv::FileStorage &fSettings);
  void LoadViewerSettings(const cv::FileStorage &fSettings);
  void LoadOrbExtractorSettings(const cv::FileStorage &fSettings);

private:
  std::vector<ImageFile> _image_files;
  GroundTruths _ground_truths;
  std::string _dataset_folder;
  cv::Mat _camera_intrinsics;
  cv::Mat _distortion_coeffs;
  ViewerSettings _viewer_settings;
  OrbExtractorSettings _orb_extractor_settings;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_DATASET_LOADER_H_
