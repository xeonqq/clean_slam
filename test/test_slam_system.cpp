//
// Created by root on 12/28/19.
//

#include "slam_system.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace clean_slam {

bool operator==(const g2o::SE3Quat &lhs, const g2o::SE3Quat &rhs) {
  const auto v1 = lhs.toVector();
  const auto v2 = rhs.toVector();
  auto result = true;
  for (size_t i = 0; i < v1.size(); ++i) {
    result &= (v1[i] == v2[i]);
  }
  return result;
}

TEST(SlamSystemTest, Run) {
  // arrange
  DatasetLoader dataset_loader;
  dataset_loader.LoadFreiburgDataset(
      DATASET_DIR + std::string("/rgbd_dataset_freiburg1_xyz"), CONFIG_DIR);
  SlamSystem system{&dataset_loader};

  // act
  system.Run();

  // assert
  const auto &trajectory = system.GetCamTrajectory();
  const auto &groundtruths = dataset_loader.GetGroundTruths();
  //  EXPECT_EQ(trajectory[0], groundtruths[0].GetTransformation());
  std::cout << trajectory[0] << std::endl;
  std::cout << groundtruths[0].GetTransformation() << std::endl;
}
} // namespace clean_slam
