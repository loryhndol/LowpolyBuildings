#pragma once

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_

#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include <unordered_map>

#include "PolygonMeshProcessing.h"
#include "Silhouette.h"
#include "calculateTau.h"

namespace LowpolyGen {

class Graph {
 private:
  std::vector<int> elements;
  int _numOfRegions;

 public:
  Graph(int numOfNodes);
  bool unite(int u, int v);
  int find(int x);
  int numOfNodes();
};

struct WeightedVector {
  Eigen::Vector3d _d;
  double _weight;

  WeightedVector() = default;
  WeightedVector(Eigen::Vector3d d, double weight) : _d(d), _weight(weight) {}

  bool operator>(const WeightedVector& rhs) const {
    return _weight > rhs._weight;
  }
};

bool EigenTupleLess(const std::tuple<double, Eigen::MatrixXd>& a,
                    const std::tuple<double, Eigen::MatrixXd>& b);

// sort based on eigenvalue
void EigenVectorOfSmallestEigenValue(Eigen::VectorXd& eigenValues,
                                     Eigen::MatrixXd& eigenVectors);

class VisualHullConstructor {
 private:
  const Config& _conf;
  std::unordered_map<int, std::vector<int>> groupTrianglesIntoRegions(
      SurfaceMesh& Mi);

  /**
   * fit a plane for each region using the L2 metric
   * part of David Cohen-Steiner, Pierre Alliez, and Mathieu Desbrun. 2004.
   * Variational Shape Approximation. ACM Trans. Graph. 23, 3 (aug 2004),
   * 905–914
   */
  std::vector<WeightedVector> fitPlanesFromRegions(
      std::vector<int>& regionHead,
      std::unordered_map<int, std::vector<int>>& regions,
      const SurfaceMesh& Mi);

  std::vector<Eigen::Vector3d> generateViewDirections(
      std::vector<WeightedVector>& weightedViewDirections, int k);

 public:
  explicit VisualHullConstructor(const Config& conf);
  SurfaceMesh run(SurfaceMesh& Mi);
  std::vector<Eigen::Vector3d> pickTopViewDirections(int k, SurfaceMesh& Mi);
};
}  // namespace LowpolyGen