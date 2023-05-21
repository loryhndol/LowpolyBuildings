#pragma once
#include <mutex>
#include <vector>

namespace LowpolyGen {
// Reference: concurrentDSU/coarseGrainedDSU
class Graph {
 private:
  std::vector<int> elements;
  int _numOfRegions;
  int _find(int u);
  std::mutex lock;

 public:
  Graph(int numOfNodes);
  bool unite(int u, int v);
  int find(int x);
  int numOfNodes();
};

}  // namespace LowpolyGen