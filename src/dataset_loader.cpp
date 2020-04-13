//
// Created by root on 12/26/19.
//

#include "dataset_loader.h"
#include <fstream>
#include <iostream>
#include <opencv2/core/persistence.hpp>
#include <sstream>
namespace clean_slam {

bool IsCommentLine(const std::string &s) { return s.find('#') == 0; }

void DatasetLoader::LoadRgb(const std::string &dataset_folder_name) {
  std::string image_inventory_filename = dataset_folder_name + "/rgb.txt";
  std::ifstream f;
  f.open(image_inventory_filename.c_str());
  while (!f.eof()) {
    std::string s;
    getline(f, s);
    if (!s.empty() && !IsCommentLine(s)) {
      std::stringstream ss;
      ss << s;
      double t;
      std::string sRGB;
      ss >> t;
      ss >> sRGB;
      _image_files.emplace_back(sRGB, t);
    }
  }
}

void DatasetLoader::LoadFreiburgGroundTruth(
    const std::string &dataset_folder_name) {

  _dataset_folder = dataset_folder_name;
  std::string gt_filename = dataset_folder_name + "/groundtruth.txt";
  std::ifstream f{gt_filename, std::ifstream::in};

  double timestamp, tx, ty, tz, qx, qy, qz, qw;
  std::string s;
  while (!f.eof()) {
    getline(f, s);
    if (!s.empty() && !IsCommentLine(s)) {
      std::stringstream ss;
      ss << s;
      ss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
      _ground_truths.emplace_back(Eigen::Vector3d(tx, ty, tz),
                                  Eigen::Quaterniond(qw, qx, qy, qz),
                                  timestamp);
    }
  }
}

void DatasetLoader::LoadFreiburgDataset(const std::string &dataset_folder_name,
                                        const std::string &path_to_yaml) {
  LoadRgb(dataset_folder_name);
  LoadFreiburgGroundTruth(dataset_folder_name);

  cv::FileStorage fSettings(path_to_yaml, cv::FileStorage::READ);
  if (fSettings.isOpened()) {

    LoadCameraIntrinsics(fSettings);
    LoadViewerSettings(fSettings);
    LoadOrbExtractorSettings(fSettings);

  } else {
    throw std::runtime_error("Could not open file: " + path_to_yaml);
  }
}

void DatasetLoader::LoadCameraIntrinsics(const cv::FileStorage &fSettings) {

  _camera_intrinsics = (cv::Mat_<double>(3, 3) << fSettings["Camera.fx"], 0,
                        fSettings["Camera.cx"], 0, fSettings["Camera.fy"],
                        fSettings["Camera.cy"], 0, 0, 1);
  _distortion_coeffs =
      (cv::Mat_<double>(5, 1) << fSettings["Camera.k1"], fSettings["Camera.k2"],
       fSettings["Camera.p1"], fSettings["Camera.p2"], fSettings["Camera.k3"]);
}

void DatasetLoader::LoadViewerSettings(const cv::FileStorage &fSettings) {

  _viewer_settings.view_point_x = fSettings["Viewer.ViewpointX"];
  _viewer_settings.view_point_y = fSettings["Viewer.ViewpointY"];
  _viewer_settings.view_point_z = fSettings["Viewer.ViewpointZ"];
  _viewer_settings.view_point_f = fSettings["Viewer.ViewpointF"];
  _viewer_settings.display_interval_ms =
      1.0f / static_cast<float>(fSettings["Camera.fps"]) * 1000.f;
}

void DatasetLoader::LoadOrbExtractorSettings(const cv::FileStorage &fSettings) {
  _orb_extractor_settings.num_features = fSettings["ORBExtractor.nFeatures"];
  _orb_extractor_settings.scale_factor = fSettings["ORBExtractor.scaleFactor"];
  _orb_extractor_settings.num_levels = fSettings["ORBExtractor.nLevels"];
}

void DatasetLoader::LoadImages(const std::string &image_folder,
                               const std::string &path_to_yaml) {
  _dataset_folder = image_folder;
  LoadRgb(image_folder);

  cv::FileStorage fSettings(path_to_yaml, cv::FileStorage::READ);
  if (fSettings.isOpened()) {
    LoadCameraIntrinsics(fSettings);
  } else {
    throw std::runtime_error("Could not open file: " + path_to_yaml);
  }
}

const std::vector<ImageFile> &DatasetLoader::GetImageFiles() const {
  return _image_files;
}

const std::vector<GroundTruth> &DatasetLoader::GetGroundTruths() const {
  return _ground_truths;
}

GroundTruth DatasetLoader::GetGroundTruthAt(double timestamp) const {
  return _ground_truths.GetGroundTruthAt(timestamp);
}

const std::string &DatasetLoader::GetDatasetFolder() const {
  return _dataset_folder;
}
const cv::Mat &DatasetLoader::GetCameraIntrinsics() const {
  return _camera_intrinsics;
}
const cv::Mat &DatasetLoader::GetDistortionCoeffs() const {
  return _distortion_coeffs;
}
const ViewerSettings &DatasetLoader::GetViewerSettings() const {
  return _viewer_settings;
}
} // namespace clean_slam
