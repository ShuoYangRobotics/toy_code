#include "transform_graph/orientation.h"

#include <gtest/gtest.h>

namespace transform_graph {
TEST(TestOrientation, Identity) {
  Orientation o;
  Eigen::Matrix3d expected;
  // clang-format off
  expected << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(o.matrix(), 0.000001));
}

TEST(TestOrientation, TestWxyz) {
  Orientation o(0.92387959, 0, 0, 0.38268353);
  Eigen::Matrix3d expected;
  // clang-format off
  expected << 0.707107, -0.707107, 0,
              0.707107,  0.707107, 0,
              0       ,  0       , 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(o.matrix(), 0.000001));
}

TEST(TestOrientation, TestEigenMatrix) {
  Eigen::Matrix3d input;
  // clang-format off
  input << 0.707107, -0.707107, 0,
           0.707107,  0.707107, 0,
           0       ,  0       , 1;
  // clang-format on

  Orientation o(input);
  Eigen::Matrix3d expected;
  // clang-format off
  expected << 0.707107, -0.707107, 0,
              0.707107,  0.707107, 0,
              0       ,  0       , 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(o.matrix(), 0.000001));
}

TEST(TestOrientation, TestEigenQuaternion) {
  Eigen::Quaterniond q(0.92387959, 0, 0, 0.38268353);
  Orientation o(q);
  Eigen::Matrix3d expected;
  // clang-format off
  expected << 0.707107, -0.707107, 0,
              0.707107,  0.707107, 0,
              0       ,  0       , 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(o.matrix(), 0.000001));
}

TEST(TestOrientation, TestGeometryMsgsQuaternion) {
  geometry_msgs::Quaternion q;
  q.w = 0.92387959;
  q.x = 0;
  q.y = 0;
  q.z = 0.38268353;
  Orientation o(q);
  Eigen::Matrix3d expected;
  // clang-format off
  expected << 0.707107, -0.707107, 0,
              0.707107,  0.707107, 0,
              0       ,  0       , 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(o.matrix(), 0.000001));
}

TEST(TestOrientation, TestTfQuaternion) {
  tf::Quaternion q(0, 0, 0.38268353, 0.92387959);
  Orientation o(q);
  Eigen::Matrix3d expected;
  // clang-format off
  expected << 0.707107, -0.707107, 0,
              0.707107,  0.707107, 0,
              0       ,  0       , 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(o.matrix(), 0.00001));
}

TEST(TestOrientation, TestTfMatrix) {
  // clang-format off
  tf::Matrix3x3 input(0.707107, -0.707107, 0,
                      0.707107,  0.707107, 0,
                      0       ,  0       , 1);
  // clang-format on

  Orientation o(input);
  Eigen::Matrix3d expected;
  // clang-format off
  expected << 0.707107, -0.707107, 0,
              0.707107,  0.707107, 0,
              0       ,  0       , 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(o.matrix(), 0.000001));
}
}  // namespace transform_graph

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
