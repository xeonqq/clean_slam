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
  EXPECT_EQ(trajectory.size(), 2);

  const auto initial_gt_transformation =
      dataset_loader.GetGroundTruthAt(trajectory.front().GetTimestamp())
          .GetTransformation();
  const auto initial_gt_transformation_inv =
      initial_gt_transformation.inverse();
  for (const auto &stamped_transformation : trajectory) {
    const g2o::SE3Quat ground_truth_transformation =
        dataset_loader.GetGroundTruthAt(stamped_transformation.GetTimestamp())
            .GetTransformation();
    const auto gt_transformation_wrt_initial_traj_pose =
        initial_gt_transformation_inv * ground_truth_transformation;
    std::printf("slame time %.6f\n", stamped_transformation.GetTimestamp());
    std::cout << "slam result: \n"
              << stamped_transformation.GetTransformation() << std::endl;
    std::cout << "ground truth: \n"
              << gt_transformation_wrt_initial_traj_pose << std::endl;
    std::cout << std::endl;
  }
  //  EXPECT_EQ(trajectory[0], groundtruths[0].GetTransformation());
}
} // namespace clean_slam
