//
// Created by root on 12/28/19.
//

#include "slam_system.h"
#include "gtest/gtest.h"

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/tuple/tuple.hpp>
#include <iostream>
#include <set>
#include <string>
#include <vector>
namespace clean_slam {

bool operator==(const g2o::SE3Quat &lhs, const g2o::SE3Quat &rhs) {
  const auto v1 = lhs.toVector();
  const auto v2 = rhs.toVector();
  auto result = true;
  for (int i = 0; i < v1.size(); ++i) {
    result &= (v1[i] == v2[i]);
  }
  return result;
}

float ComputeTransformationError(const g2o::SE3Quat &ground_truth,
                                 const StampedTransformation &calculated) {
  const auto calculated_rotation =
      calculated.GetTransformation().to_homogeneous_matrix().block<3, 3>(0, 0);
  const auto calculated_euler_angle = calculated_rotation.eulerAngles(0, 1, 2);
  const auto calculated_translation =
      calculated.GetTransformation().to_homogeneous_matrix().block<3, 1>(0, 3);

  const auto gt_rotation =
      ground_truth.to_homogeneous_matrix().block<3, 3>(0, 0);
  const auto gt_euler_angle = gt_rotation.eulerAngles(0, 1, 2);
  const auto gt_translation =
      ground_truth.to_homogeneous_matrix().block<3, 1>(0, 3);
  return (gt_translation - calculated_translation).norm();
}

TEST(SlamSystemFreiburgTest, Run) {
  // arrange
  DatasetLoader dataset_loader;
  dataset_loader.LoadFreiburgDataset(
      DATASET_DIR + std::string("/rgbd_dataset_freiburg1_xyz"),
      CONFIG_DIR + std::string("/TUM1.yaml"));
  SlamSystem system{&dataset_loader};

  // act
  system.Run();

  // assert
  const auto &trajectory = system.GetCamTrajectory();
  std::cout << "trajectory size: " << trajectory.size() << std::endl;
  //  EXPECT_EQ(trajectory.size(), 2);

  const auto initial_gt_transformation =
      dataset_loader.GetGroundTruthAt(trajectory.front().GetTimestamp())
          .GetTransformation();
  for (const auto &stamped_transformation : trajectory) {
    const g2o::SE3Quat ground_truth_transformation =
        dataset_loader.GetGroundTruthAt(stamped_transformation.GetTimestamp())
            .GetTransformation();
    const auto gt_initial_pose_wrt_current_pose =
        ground_truth_transformation * initial_gt_transformation.inverse();
    //    std::printf("slam time %.6f\n",
    //    stamped_transformation.GetTimestamp());
    std::cout << "slam result: \n"
              << stamped_transformation.GetTransformation() << std::endl;
    const auto rotation = stamped_transformation.GetTransformation()
                              .to_homogeneous_matrix()
                              .block<3, 3>(0, 0);
    const auto euler_angles = rotation.eulerAngles(0, 1, 2);
    const auto translation = stamped_transformation.GetTransformation()
                                 .to_homogeneous_matrix()
                                 .block<3, 1>(0, 3);

    std::cout << "ground truth: \n"
              << gt_initial_pose_wrt_current_pose << std::endl;
    std::cout << "translation error: \n"
              << ComputeTransformationError(gt_initial_pose_wrt_current_pose,
                                            stamped_transformation)
              << std::endl;
    std::cout << "euler angle calculated: \n";
    std::cout << euler_angles << std::endl;

    const auto rotation_gt =
        gt_initial_pose_wrt_current_pose.to_homogeneous_matrix().block<3, 3>(0,
                                                                             0);
    const auto translation_gt =
        gt_initial_pose_wrt_current_pose.to_homogeneous_matrix().block<3, 1>(0,
                                                                             3);
    const auto euler_angles_gt = rotation_gt.eulerAngles(0, 1, 2);

    //    float scale = 5.524032482088845;
    //    for (size_t i = 0; i < 3; ++i) {
    //      EXPECT_NAER(euler_angles_gt[i], euler_angles[i], );
    //      EXPECT_NEAR(translation_gt[i], translation[i], 1e-5);
    //    }
    std::cout << "euler angle gt: \n";
    std::cout << euler_angles_gt << std::endl;
    std::cout << std::endl;
  }
  //  EXPECT_EQ(trajectory[0], groundtruths[0].GetTransformation());
}

} // namespace clean_slam
