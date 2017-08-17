#ifndef _TRANSFORM_GRAPH_GRAPH_INTERNAL_H_
#define _TRANSFORM_GRAPH_GRAPH_INTERNAL_H_

#include <map>
#include <set>
#include <string>
#include <vector>

namespace transform_graph {
namespace internal {
/// \brief Graph implements a sparse, unweighted, and undirected
/// graph.
class Graph {
 public:
  Graph();

  /// \brief Adds an undirected edge to the graph.
  void AddEdge(const std::string& v1, const std::string& v2);

  /// \brief Finds the path between the source vertex and the target vertex.
  /// If a path exists, it is specified in the path vector (including both
  /// endpoints). If the source and target are the same, then the path returned
  /// is of length 1.
  /// \returns true if a path exists, false otherwise.
  bool Path(const std::string& source, const std::string& target,
            std::vector<std::string>* path) const;

  /// \brief Returns if the given vertex exists in the graph.
  /// \returns true if the given vertex exists in the graph, false otherwise.
  bool HasVertex(const std::string& v) const;

 private:
  std::map<std::string, std::set<std::string> > adjacencies_;
};
}  // namespace internal
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_GRAPH_INTERNAL_H_
