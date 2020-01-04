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

void DatasetLoader::LoadFreiburgRgb(const std::string &dataset_folder_name) {
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
  LoadFreiburgRgb(dataset_folder_name);
  LoadFreiburgGroundTruth(dataset_folder_name);
  LoadFreiburgCameraIntrinsics(path_to_yaml);
}

void DatasetLoader::LoadFreiburgCameraIntrinsics(
    const std::string &path_to_yaml) {
  cv::FileStorage fSettings(path_to_yaml + "/TUM1.yaml", cv::FileStorage::READ);
  _camera_intrinsics = (cv::Mat_<double>(3, 3) << fSettings["Camera.fx"], 0,
                        fSettings["Camera.cx"], 0, fSettings["Camera.fy"],
                        fSettings["Camera.cy"], 0, 0, 1);
  _distortion_coeffs =
      (cv::Mat_<double>(5, 1) << fSettings["Camera.k1"], fSettings["Camera.k2"],
       fSettings["Camera.p1"], fSettings["Camera.p2"], fSettings["Camera.k3"]);

  //  const float k3 = fSettings["Camera.k3"];
  //  if (k3 != 0) {
  //    _distortion_coeffs.resize(5, k3);
  //  }
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
} // namespace clean_slam
