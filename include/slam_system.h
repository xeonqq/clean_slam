//
// Created by root on 12/25/19.
//

#ifndef CLEAN_SLAM_SLAMSYSTEM_H
#define CLEAN_SLAM_SLAMSYSTEM_H

#include "dataset_loader.h"

namespace clean_slam {
class SlamSystem {
public:
  SlamSystem();
  void LoadMonoDataset(const std::string &dataset_folder,
                       const std::string &path_to_yaml);
  void Run();
  const std::vector<g2o::SE3Quat> &GetCamTrajectory() const;

private:
  DatasetLoader _dataset_loader;

  std::vector<g2o::SE3Quat> _cam_trajectory;
};
}

#endif // CLEAN_SLAM_SLAMSYSTEM_H
