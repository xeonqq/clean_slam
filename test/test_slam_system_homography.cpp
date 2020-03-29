//
// Created by root on 12/28/19.
//

#include "slam_system.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <cv.hpp>

namespace clean_slam {

TEST(SlamSystemHomographyTest, DISABLED_Run) {
  // arrange
  DatasetLoader dataset_loader;
  dataset_loader.LoadImages(TEST_DIR + std::string("/huawei_640x480"),
                            CONFIG_DIR + std::string("/Huawei.yaml"));
  SlamSystem system{&dataset_loader};

  // act
  system.Run();

  // assert
  const auto &trajectory = system.GetCamTrajectory();
  EXPECT_EQ(trajectory.size(), 2);

  for (const auto &stamped_transformation : trajectory) {
    std::cout << "slam result: \n"
              << stamped_transformation.GetTransformation() << std::endl;
    const auto rotation = stamped_transformation.GetTransformation()
                              .to_homogeneous_matrix()
                              .block<3, 3>(0, 0);
    const auto euler_angles = rotation.eulerAngles(0, 1, 2);
    std::cout << euler_angles << std::endl;
    std::cout << std::endl;
  }
  //  EXPECT_EQ(trajectory[0], groundtruths[0].GetTransformation());
}
} // namespace clean_slam
