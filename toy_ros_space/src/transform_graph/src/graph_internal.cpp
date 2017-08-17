#include "transform_graph/graph_internal.h"

#include <algorithm>
#include <map>
#include <queue>
#include <set>
#include <string>

using std::map;
using std::queue;
using std::set;
using std::string;

namespace transform_graph {
namespace internal {
Graph::Graph() : adjacencies_() {}

void Graph::AddEdge(const std::string& v1, const std::string& v2) {
  adjacencies_[v1].insert(v2);
  adjacencies_[v2].insert(v1);
}

bool Graph::Path(const std::string& source, const std::string& target,
                 std::vector<std::string>* path) const {
  path->clear();
  if (!HasVertex(source) || !HasVertex(target)) {
    return false;
  }

  // Used to backtrack the path once the target has been found. Also used to
  // track whether a node has been visited or not. If a node has no parent, it
  // is either the source node or it has not been visited yet.
  map<string, string> parents;

  queue<string> queue;
  queue.push(source);
  while (!queue.empty()) {
    const string& current = queue.front();

    if (current == target) {
      string cur = target;
      path->push_back(cur);
      while (cur != source && parents.find(cur) != parents.end()) {
        cur = parents[cur];
        path->push_back(cur);
      }
      std::reverse(path->begin(), path->end());
      return true;
    }

    queue.pop();
    const set<string>& neighbors = adjacencies_.at(current);
    for (set<string>::iterator it = neighbors.begin(); it != neighbors.end();
         ++it) {
      string neighbor(it->data());
      if (neighbor != source && parents.find(neighbor) == parents.end()) {
        parents[neighbor] = current;
        queue.push(neighbor);
      }
    }
  }

  return false;
}

bool Graph::HasVertex(const std::string& v) const {
  return adjacencies_.find(v) != adjacencies_.end();
}
}  // namespace internal
}  // namespace transform_graph
