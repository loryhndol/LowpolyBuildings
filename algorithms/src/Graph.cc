#include "engine/Graph.h"

namespace LowpolyGen {
Graph::Graph(int numOfNodes) : _numOfRegions(numOfNodes) {
  elements.assign(_numOfRegions, -1);
}

bool Graph::unite(int u, int v) {
  std::lock_guard<std::mutex> lg(lock);  // lock before mutating
  u = _find(u);
  v = _find(v);
  if (u == v) return false;
  if (elements[u] > elements[v]) std::swap(u, v);  // union by rank
  if (elements[u] == elements[v]) --elements[u];
  elements[v] = u;
  --_numOfRegions;
  return true;
}

int Graph::find(int u) {
  std::lock_guard<std::mutex> lg(lock);
  int p = _find(u);
  return p;
}

int Graph::numOfNodes() { return elements.size(); }

int Graph::_find(int u) {
  if (elements[u] < 0) return u;
  elements[u] = _find(elements[u]);  // path compression
  return elements[u];
}
}  // namespace LowpolyGen