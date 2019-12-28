//
// Created by root on 12/25/19.
//

#include "slam_system.h"
#include <cv.hpp>
#include <opencv2/imgcodecs.hpp>

namespace clean_slam {

SlamSystem::SlamSystem() {}
void SlamSystem::LoadMonoDataset(const std::string &dataset_folder,
                                 const std::string &path_to_yaml) {
  _dataset_loader.LoadFreiburgDataset(dataset_folder, path_to_yaml);
}

const std::vector<g2o::SE3Quat> &SlamSystem::GetCamTrajectory() const {
  return _cam_trajectory;
}
void SlamSystem::Run() {
  for (const auto &image_file : _dataset_loader.GetImageFiles()) {
    auto im = cv::imread(_dataset_loader.GetDatasetFolder() + '/' +
                             image_file.image_filename,
                         cv::IMREAD_GRAYSCALE);
//    cv::imshow("image", im);
//    cv::waitKey(0);
  }
}
} // namespace clean_slam
