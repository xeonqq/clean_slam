//
// Created by root on 12/28/19.
//

#include "slam_system.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace clean_slam {

TEST(SlamSystemTest, Run) {
  // arrange
  SlamSystem system;
  // act
  system.LoadMonoDataset(
      DATASET_DIR + std::string("/rgbd_dataset_freiburg1_xyz"), CONFIG_DIR);
  system.Run();
}
}
