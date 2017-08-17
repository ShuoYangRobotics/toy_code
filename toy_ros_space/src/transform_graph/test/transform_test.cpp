#include "transform_graph/transform.h"

#include <gtest/gtest.h>

namespace transform_graph {
TEST(TestTransform, TestDefaultIsIdentity) {
  Transform t;
  Eigen::Matrix4d expected;
  // clang-format off
  expected << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(t.matrix(), 0.000001));
}

TEST(TestTransform, TestIdentity) {
  Transform t = Transform::Identity();
  Eigen::Matrix4d expected;
  // clang-format off
  expected << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(t.matrix(), 0.000001));
}

TEST(TestTransform, TestPosOri) {
  transform_graph::Position pos(10, 5, 0);
  Eigen::Matrix3d rot;
  // clang-format off
  rot << 0.86603, -0.5,     0,
         0.5,      0.86603, 0,
         0,        0,       1;
  // clang-format on
  transform_graph::Orientation ori(rot);
  Transform t(pos, ori);

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.86603, -0.5,     0, 10,
              0.5,      0.86603, 0, 5,
              0,        0,       1, 0,
              0,        0,       0, 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(t.matrix(), 0.00001));
}

TEST(TestTransform, TestTfTransform) {
  // clang-format off
  tf::Matrix3x3 rot(0.86603, -0.5,     0,
                    0.5,      0.86603, 0,
                    0,        0,       1);
  // clang-format on
  tf::Vector3 pos(10, 5, 0);
  tf::Transform trans(rot, pos);
  Transform t(trans);

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.86603, -0.5,     0, 10,
              0.5,      0.86603, 0, 5,
              0,        0,       1, 0,
              0,        0,       0, 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(t.matrix(), 0.000001));
}

TEST(TestTransform, TestGeometryMsgsPose) {
  geometry_msgs::Pose pose;
  pose.orientation.w = 0.96592702;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0.25881873;
  pose.position.x = 10;
  pose.position.y = 5;
  Transform t(pose);

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.86603, -0.5,     0, 10,
              0.5,      0.86603, 0, 5,
              0,        0,       1, 0,
              0,        0,       0, 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(t.matrix(), 0.000001));
}

TEST(TestTransform, TestEigenAffine) {
  Eigen::Matrix4d input;
  // clang-format off
  input << 0.86603, -0.5,     0, 10,
           0.5,      0.86603, 0, 5,
           0,        0,       1, 0,
           0,        0,       0, 1;
  // clang-format on
  Eigen::Affine3d affine(input);
  Transform t(affine);
  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.86603, -0.5,     0, 10,
              0.5,      0.86603, 0, 5,
              0,        0,       1, 0,
              0,        0,       0, 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(t.matrix(), 0.000001));
}

TEST(TestTransform, TestEigenMatrix) {
  Eigen::Matrix4d input;
  // clang-format off
  input << 0.86603, -0.5,     0, 10,
           0.5,      0.86603, 0, 5,
           0,        0,       1, 0,
           0,        0,       0, 1;
  // clang-format on
  Transform t(input);
  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.86603, -0.5,     0, 10,
              0.5,      0.86603, 0, 5,
              0,        0,       1, 0,
              0,        0,       0, 1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(t.matrix(), 0.000001));
}

TEST(TestTransform, TestInverse) {
  Eigen::Matrix4d input;
  // clang-format off
  input << 0.866, -0.5,  0, 4,
           0.5,   0.866, 0, 3,
           0,     0,     1, 0,
           0,     0,     0, 1;
  // clang-format on
  Transform t(input);
  Eigen::Matrix4d expected;
  // clang-format off
  expected <<  0.866, 0.5,   0, -4.964,
              -0.5,   0.866, 0, -0.598,
               0,     0,     1,  0,
               0,     0,     0,  1;
  // clang-format on
  EXPECT_TRUE(expected.isApprox(t.inverse().matrix(), 0.0001));
}
}  // namespace transform_graph

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
