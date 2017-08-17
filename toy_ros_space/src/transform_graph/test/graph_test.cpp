#include "transform_graph/graph.h"

#include <string>

#include "Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "gtest/gtest.h"

namespace transform_graph {
// Basic connectivity tests
TEST(TestGraph, InitiallyEmpty) {
  Graph graph;
  EXPECT_FALSE(graph.CanTransform("A", "B"));
  EXPECT_FALSE(graph.CanTransform("foo", "foo bar"));
}

TEST(TestGraph, SimpleConnection) {
  Graph graph;
  graph.Add("tool", RefFrame("base link"), Transform());
  EXPECT_FALSE(graph.CanTransform("tool", "asdf"));
  EXPECT_TRUE(graph.CanTransform("tool", "base link"));
  EXPECT_TRUE(graph.CanTransform("base link", "tool"));
}

TEST(TestGraph, SelfTarget) {
  Graph graph;
  graph.Add("tool", RefFrame("base link"), Transform());
  EXPECT_TRUE(graph.CanTransform("tool", "tool"));
}

TEST(TestGraph, LinkedList) {
  Graph graph;
  graph.Add("B", RefFrame("A"), Transform());
  graph.Add("C", RefFrame("B"), Transform());
  EXPECT_TRUE(graph.CanTransform("A", "C"));
  EXPECT_TRUE(graph.CanTransform("C", "A"));
}

TEST(TestGraph, DisconnectedSuccess) {
  Graph graph;
  graph.Add("B", RefFrame("A"), Transform());
  graph.Add("C", RefFrame("D"), Transform());
  EXPECT_TRUE(graph.CanTransform("A", "B"));
}

TEST(TestGraph, DisconnectedFail) {
  Graph graph;
  graph.Add("B", RefFrame("A"), Transform());
  graph.Add("C", RefFrame("D"), Transform());
  EXPECT_FALSE(graph.CanTransform("A", "C"));
}

TEST(TestGraph, SelfTargetIsIdentity) {
  Graph graph;
  Eigen::Vector3d translation(1, 2, 3);
  graph.Add("tool", RefFrame("base_link"),
            Transform(translation, Orientation()));

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
  // clang-format on
  Transform actual;
  bool success =
      graph.ComputeDescription(LocalFrame("tool"), RefFrame("tool"), &actual);
  EXPECT_TRUE(success);

  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.000001));
}

// Simple tests
TEST(TestGraph, SimpleTranslationDescription) {
  Graph graph;
  Eigen::Vector3d translation(1, 2, 3);
  Orientation rotation;
  graph.Add("some link", RefFrame("base link"),
            Transform(translation, rotation));

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 1, 0, 0, 1,
              0, 1, 0, 2,
              0, 0, 1, 3,
              0, 0, 0, 1;
  // clang-format on
  Transform actual;
  bool success = graph.ComputeDescription(LocalFrame("some link"),
                                          RefFrame("base link"), &actual);
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.000001));

  // Reverse direction
  expected(0, 3) = -1;
  expected(1, 3) = -2;
  expected(2, 3) = -3;
  graph.ComputeDescription(LocalFrame("base link"), RefFrame("some link"),
                           &actual);
  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.000001));
}

TEST(TestGraph, SimpleOrientationDescription) {
  Graph graph;
  Position translation;
  Eigen::Matrix3d rot_matrix;
  // clang-format off
  rot_matrix << 0.866, -0.5,  0,
                0.5,   0.866, 0,
                0,     0,     1;
  // clang-format on

  graph.Add("some link", RefFrame("base link"),
            Transform(translation, rot_matrix));

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.866, -0.5,  0, 0,
              0.5,   0.866, 0, 0,
              0,     0,     1, 0,
              0,     0,     0, 1;
  // clang-format on
  Transform actual;
  bool success = graph.ComputeDescription(LocalFrame("some link"),
                                          RefFrame("base link"), &actual);
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.000001));

  // Reverse direction
  expected.topLeftCorner(3, 3).transposeInPlace();
  graph.ComputeDescription(LocalFrame("base link"), RefFrame("some link"),
                           &actual);
  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.0001));
}

TEST(TestGraph, TranslationChainDescription) {
  Graph graph;
  Eigen::Vector3d base_torso(0, 0, 3);
  Eigen::Vector3d torso_tool(0.1, 0.2, 0.3);
  Orientation identity_rot;
  graph.Add("torso", RefFrame("base"), Transform(base_torso, identity_rot));
  graph.Add("tool", RefFrame("torso"), Transform(torso_tool, identity_rot));

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 1, 0, 0, 0.1,
              0, 1, 0, 0.2,
              0, 0, 1, 3.3,
              0, 0, 0, 1;
  // clang-format on
  Transform actual;
  bool success =
      graph.ComputeDescription(LocalFrame("tool"), RefFrame("base"), &actual);
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.000001));

  // Reverse direction
  expected(0, 3) = -0.1;
  expected(1, 3) = -0.2;
  expected(2, 3) = -3.3;
  graph.ComputeDescription(LocalFrame("base"), RefFrame("tool"), &actual);
  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.000001));
}

TEST(TestGraph, OrientationChainDescription) {
  Graph graph;
  Position identity;
  Eigen::Matrix3d ori12;
  Eigen::Matrix3d ori23;
  // clang-format off
  ori12 << 0.866, -0.5,  0,
           0.5,   0.866, 0,
           0,     0,     1;
  ori23 << 1, 0,     0,
           0, 0.866, 0.5,
           0, 0.5,   0.866;
  // clang-format on

  graph.Add("link1", RefFrame("base"), Transform(identity, ori12));
  graph.Add("link2", RefFrame("link1"), Transform(identity, ori23));

  Eigen::Matrix3d ori13 = ori12 * ori23;
  Eigen::Affine3d affine13(ori13);
  Eigen::Matrix3d ori31 = ori23.inverse() * ori12.inverse();
  Eigen::Affine3d affine31(ori31);

  Transform actual;
  bool success =
      graph.ComputeDescription(LocalFrame("link2"), RefFrame("base"), &actual);
  EXPECT_TRUE(success);
  EXPECT_TRUE(affine13.matrix().isApprox(actual.matrix(), 0.000001));

  // Reverse direction
  graph.ComputeDescription(LocalFrame("base"), RefFrame("link2"), &actual);
  EXPECT_TRUE(affine31.matrix().isApprox(actual.matrix(), 0.000001));
}

TEST(TestGraph, InverseDescription) {
  Graph graph;
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix << 0.866, -0.5,  0, 4,
            0.5,   0.866, 0, 3,
            0,     0,     1, 0,
            0,     0,     0, 1;
  // clang-format on
  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.866, 0.5,   0, -4.964,
              -0.5,  0.866, 0, -0.598,
              0,     0,     1, 0,
              0,     0,     0, 1;
  // clang-format on

  graph.Add("B", RefFrame("A"), matrix);

  Transform actual;
  bool success =
      graph.ComputeDescription(LocalFrame("A"), RefFrame("B"), &actual);
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.0001));
}

TEST(TestGraph, DescribePoint) {
  Graph graph;
  Eigen::Matrix4d b_matrix;
  // clang-format off
  b_matrix << 0.866, -0.5,  0, 10,
              0.5,   0.866, 0, 5,
              0,     0,     1, 0,
              0,     0,     0, 1;
  // clang-format on
  graph.Add("B", RefFrame("A"), b_matrix);

  geometry_msgs::Point pt_in_b;
  pt_in_b.x = 3;
  pt_in_b.y = 7;
  pt_in_b.z = 0;

  Eigen::Vector3d expected(9.098, 12.562, 0);

  Position actual;
  bool success =
      graph.DescribePosition(pt_in_b, Source("B"), Target("A"), &actual);
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.vector(), 0.000001));
}

TEST(TestGraph, DescribePose) {
  Graph graph;
  Eigen::Matrix4d tool_in_base;
  // clang-format off
  tool_in_base << 0.866, -0.5, 0, 10,
                  0.5,   0.866, 0, 5,
                  0,     0,     1, 0,
                  0,     0,     0, 1;
  // clang-format on

  graph.Add("tool", RefFrame("base"), tool_in_base);

  geometry_msgs::Pose pose_in_tool;
  pose_in_tool.orientation.w = 1;
  pose_in_tool.position.x = 3;
  pose_in_tool.position.y = 7;
  pose_in_tool.position.z = 0;

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.866, -0.5,  0, 9.098,
              0.5,   0.866, 0, 12.562,
              0,     0,     1, 0,
              0,     0,     0, 1;
  // clang-format on

  Transform actual;
  bool success =
      graph.DescribePose(pose_in_tool, Source("tool"), Target("base"), &actual);
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.matrix(), 0.000001));
}

// Some simple tests for the mapping methods.
TEST(TestGraph, ComputeSimpleMapping) {
  Graph graph;
  Eigen::Vector3d left_hand(0, 0.5, 0);
  Eigen::Vector3d right_hand(0, -0.5, 0);
  Orientation identity;

  graph.Add("left_hand", RefFrame("base"), Transform(left_hand, identity));
  graph.Add("right_hand", RefFrame("base"), Transform(right_hand, identity));

  Transform actual;
  bool success =
      graph.ComputeMapping(From("left_hand"), To("right_hand"), &actual);
  Eigen::Vector3d expected(0, -1, 0);
  EXPECT_TRUE(success);
  EXPECT_TRUE(
      expected.isApprox(actual.matrix().topRightCorner(3, 1), 0.000001));

  // Reverse
  success = graph.ComputeMapping(From("right_hand"), To("left_hand"), &actual);
  expected.y() = 1;
  EXPECT_TRUE(success);
  EXPECT_TRUE(
      expected.isApprox(actual.matrix().topRightCorner(3, 1), 0.000001));
}

TEST(TestGraph, SimpleMapPosition) {
  Graph graph;
  Eigen::Vector3d left_hand(0, 0.5, 0);
  Eigen::Vector3d right_hand(0, -0.5, 0);
  Orientation identity;

  graph.Add("left_hand", RefFrame("base"), Transform(left_hand, identity));
  graph.Add("right_hand", RefFrame("base"), Transform(right_hand, identity));

  Position actual;
  geometry_msgs::Point pt;
  pt.x = 1;
  pt.y = 0;
  pt.z = 0;
  bool success =
      graph.MapPosition(pt, From("left_hand"), To("right_hand"), &actual);
  Eigen::Vector3d expected(1, -1, 0);
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.vector(), 0.000001));

  // Reverse
  success = graph.MapPosition(pt, From("right_hand"), To("left_hand"), &actual);
  expected.y() = 1;
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.vector(), 0.000001));
}

TEST(TestGraph, MapPose) {
  Graph graph;
  Eigen::Vector3d left_hand(0, 1, 0);
  Eigen::Vector3d right_hand(0, -1, 0);
  Orientation identity;

  graph.Add("left_hand", RefFrame("base"), Transform(left_hand, identity));
  graph.Add("right_hand", RefFrame("base"), Transform(right_hand, identity));

  Eigen::Matrix3d rotation(
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 0, 1)));
  Eigen::Quaterniond quaternion(rotation.matrix());

  Transform actual;
  geometry_msgs::Pose pose;
  pose.position.x = 1;
  pose.position.y = 0;
  pose.position.z = 0;
  pose.orientation.w = quaternion.w();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  bool success =
      graph.MapPose(pose, From("left_hand"), To("right_hand"), &actual);

  Eigen::Matrix4d expected;
  // clang-format off
  expected << 0.707107, -0.707107, 0, 1,
              0.707107,  0.707107, 0, -2,
              0,         0,        1, 0,
              0,         0,        0, 1;
  // clang-format on

  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.matrix().isApprox(actual.matrix(), 0.000001));
}

TEST(TestGraph, ComputeMappingBetweenRotatedFrames) {
  Graph graph;
  Eigen::Vector3d left_hand(0, 0.5, 0);
  Eigen::Vector3d right_hand(0, -0.5, 0);
  Orientation identity;

  Eigen::Matrix3d rotation(
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 0, 1)));
  graph.Add("left_hand", RefFrame("base"), Transform(left_hand, identity));
  graph.Add("right_hand", RefFrame("base"), Transform(right_hand, rotation));

  geometry_msgs::Point point;
  point.x = sqrt(2);
  point.y = 0;
  point.z = 0;
  Eigen::Vector4d eigen_point;
  eigen_point << point.x, point.y, point.z, 1;

  Eigen::Vector3d expected;
  expected << 1, 0, 0;

  transform_graph::Transform left_to_right;
  bool success =
      graph.ComputeMapping(From("left_hand"), To("right_hand"), &left_to_right);
  Eigen::Vector3d compute_actual =
      (left_to_right.matrix() * eigen_point).head<3>();
  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(compute_actual, 0.000001));
}

TEST(TestGraph, MapPositionBetweenRotatedFrames) {
  Graph graph;
  Eigen::Vector3d left_hand(0, 0.5, 0);
  Eigen::Vector3d right_hand(0, -0.5, 0);
  Orientation identity;

  Eigen::Matrix3d rotation(
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 0, 1)));
  graph.Add("left_hand", RefFrame("base"), Transform(left_hand, identity));
  graph.Add("right_hand", RefFrame("base"), Transform(right_hand, rotation));

  geometry_msgs::Point point;
  point.x = sqrt(2);
  point.y = 0;
  point.z = 0;
  Eigen::Vector4d eigen_point;
  eigen_point << point.x, point.y, point.z, 1;

  Eigen::Vector3d expected;
  expected << 1, 0, 0;

  Position actual;
  bool success =
      graph.MapPosition(point, From("left_hand"), To("right_hand"), &actual);

  EXPECT_TRUE(success);
  EXPECT_TRUE(expected.isApprox(actual.vector(), 0.000001));
}
}  // namespace transform_graph

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
