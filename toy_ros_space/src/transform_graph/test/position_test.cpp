#include "transform_graph/position.h"

#include <gtest/gtest.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_types.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"

namespace transform_graph {
TEST(TestPosition, Identity) {
  Position p;
  EXPECT_EQ(p.vector().x(), 0);
  EXPECT_EQ(p.vector().y(), 0);
  EXPECT_EQ(p.vector().z(), 0);
}

TEST(TestPosition, TestXyz) {
  Position p(1, 2, 3);
  EXPECT_EQ(p.vector().x(), 1);
  EXPECT_EQ(p.vector().y(), 2);
  EXPECT_EQ(p.vector().z(), 3);
}

TEST(TestPosition, TestEigen) {
  Eigen::Vector3d v(1, 2, 3);
  Position p(v);
  EXPECT_EQ(p.vector().x(), 1);
  EXPECT_EQ(p.vector().y(), 2);
  EXPECT_EQ(p.vector().z(), 3);
}

TEST(TestPosition, TestPoint) {
  geometry_msgs::Point p;
  p.x = 1;
  p.y = 2;
  p.z = 3;
  Position pos(p);
  EXPECT_EQ(pos.vector().x(), 1);
  EXPECT_EQ(pos.vector().y(), 2);
  EXPECT_EQ(pos.vector().z(), 3);
}

TEST(TestPosition, TestGeoVector) {
  geometry_msgs::Vector3 v;
  v.x = 1;
  v.y = 2;
  v.z = 3;
  Position pos(v);
  EXPECT_EQ(pos.vector().x(), 1);
  EXPECT_EQ(pos.vector().y(), 2);
  EXPECT_EQ(pos.vector().z(), 3);
}

TEST(TestPosition, TestPclXyz) {
  pcl::PointXYZ v;
  v.x = 1;
  v.y = 2;
  v.z = 3;
  Position pos(v);
  EXPECT_EQ(pos.vector().x(), 1);
  EXPECT_EQ(pos.vector().y(), 2);
  EXPECT_EQ(pos.vector().z(), 3);
}

TEST(TestPosition, TestTfVector) {
  tf::Vector3 v(1, 2, 3);
  Position pos(v);
  EXPECT_EQ(pos.vector().x(), 1);
  EXPECT_EQ(pos.vector().y(), 2);
  EXPECT_EQ(pos.vector().z(), 3);
}
}  // namespace transform_graph

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
