//
// Created by root on 12/25/19.
//

#include "dataset_loader.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace clean_slam {

TEST(DatasetLoaderTest, load_dataset) {
  // arrange
  DatasetLoader loader;
  // act
  loader.LoadFreiburgDataset(DATASET_DIR +
                             std::string("/rgbd_dataset_freiburg1_xyz"));
  const auto& image_files = loader.GetImageFiles();
  const auto& ground_truths = loader.GetGroundTruths();

  // assert
  EXPECT_EQ(image_files.size(), 798);
  EXPECT_DOUBLE_EQ(image_files.front().timestamp, 1305031102.175304);
  EXPECT_EQ(image_files.front().image_filename, "rgb/1305031102.175304.png");

  EXPECT_EQ(ground_truths.size(), 3000);

  EXPECT_EQ(ground_truths.front().GetTimestamp(), 1305031098.6659);
  const auto& q = ground_truths.front().GetQuaternion() ;
  EXPECT_NEAR(-q.x(), 0.6132,1.1e-4);
  EXPECT_NEAR(-q.y(), 0.5962, 1.1e-4);
  EXPECT_NEAR(-q.z(), -0.331, 1.1e-4);
  EXPECT_NEAR(-q.w(), -0.3986,1.1e-4);
  EXPECT_TRUE(ground_truths.front().GetTranslation().isApprox(
      Eigen::Vector3d(1.3563, 0.6305, 1.6380)));
}
} // namespace clean_slam
