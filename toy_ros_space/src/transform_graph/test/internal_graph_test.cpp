#include "transform_graph/graph_internal.h"

#include <string>
#include <vector>
#include <gtest/gtest.h>

using std::string;
using std::vector;

namespace transform_graph {
namespace internal {
TEST(TestGraphInternal, EmptyGraphFails) {
  Graph graph;
  vector<string> path;
  bool success = graph.Path("base_link", "tool", &path);
  EXPECT_FALSE(graph.HasVertex("base_link"));
  EXPECT_FALSE(graph.HasVertex("tool"));
  EXPECT_FALSE(success);
}

TEST(TestGraphInternal, SimpleCase) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("base_link", "tool");
  bool success = graph.Path("base_link", "tool", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(2, path.size());
  EXPECT_EQ("base_link", path[0]);
  EXPECT_EQ("tool", path[1]);
}

TEST(TestGraphInternal, BackwardsSimpleCase) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("base_link", "tool");
  bool success = graph.Path("tool", "base_link", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(2, path.size());
  EXPECT_EQ("tool", path[0]);
  EXPECT_EQ("base_link", path[1]);
}

TEST(TestGraphInternal, SingleNodeSelfEdge) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("base_link", "base_link");
  bool success = graph.Path("base_link", "base_link", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(1, path.size());
  EXPECT_EQ("base_link", path[0]);
}

TEST(TestGraphInternal, SourceTargetSame) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("base_link", "tool");
  bool success = graph.Path("base_link", "base_link", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(1, path.size());
  EXPECT_EQ("base_link", path[0]);
}

TEST(TestGraphInternal, LinkedList) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("A", "B");
  graph.AddEdge("B", "C");
  graph.AddEdge("C", "D");
  bool success = graph.Path("B", "D", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(3, path.size());
  EXPECT_EQ("B", path[0]);
  EXPECT_EQ("C", path[1]);
  EXPECT_EQ("D", path[2]);
}

TEST(TestGraphInternal, BackwardsLinkedList) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("A", "B");
  graph.AddEdge("B", "C");
  graph.AddEdge("C", "D");
  bool success = graph.Path("D", "A", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(4, path.size());
  EXPECT_EQ("D", path[0]);
  EXPECT_EQ("C", path[1]);
  EXPECT_EQ("B", path[2]);
  EXPECT_EQ("A", path[3]);
}

TEST(TestGraphInternal, DiamondBisected) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("A", "B");
  graph.AddEdge("A", "C");
  graph.AddEdge("B", "C");
  graph.AddEdge("B", "D");
  graph.AddEdge("C", "D");
  bool success = graph.Path("A", "D", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(3, path.size());
  EXPECT_EQ("A", path[0]);
  EXPECT_TRUE(path[1] == "B" || path[1] == "C");
  EXPECT_EQ("D", path[2]);
}

TEST(TestGraphInternal, ShortestPath) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("A", "B");
  graph.AddEdge("B", "C");
  graph.AddEdge("C", "D");
  graph.AddEdge("D", "E");
  graph.AddEdge("E", "A");
  bool success = graph.Path("A", "C", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(3, path.size());
  EXPECT_EQ("A", path[0]);
  EXPECT_EQ("B", path[1]);
  EXPECT_EQ("C", path[2]);
}

TEST(TestGraphInternal, ShortestPathBackwards) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("A", "B");
  graph.AddEdge("B", "C");
  graph.AddEdge("C", "D");
  graph.AddEdge("D", "E");
  graph.AddEdge("E", "A");
  bool success = graph.Path("A", "D", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(3, path.size());
  EXPECT_EQ("A", path[0]);
  EXPECT_EQ("E", path[1]);
  EXPECT_EQ("D", path[2]);
}

TEST(TestGraphInternal, DisconnectedFail) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("A", "B");
  graph.AddEdge("C", "D");
  bool success = graph.Path("A", "C", &path);
  EXPECT_FALSE(success);
}

TEST(TestGraphInternal, DisconnectedSuccess) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("A", "B");
  graph.AddEdge("C", "D");
  bool success = graph.Path("A", "B", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(2, path.size());
  EXPECT_EQ("A", path[0]);
  EXPECT_EQ("B", path[1]);
}

TEST(TestGraphInternal, MiddleSelfLoop) {
  Graph graph;
  vector<string> path;
  graph.AddEdge("A", "B");
  graph.AddEdge("B", "B");
  graph.AddEdge("B", "C");
  bool success = graph.Path("C", "A", &path);
  EXPECT_TRUE(success);
  ASSERT_EQ(3, path.size());
  EXPECT_EQ("C", path[0]);
  EXPECT_EQ("B", path[1]);
  EXPECT_EQ("A", path[2]);
}
}  // namespace internal
}  // namespace transform_graph

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
