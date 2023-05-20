#ifndef _CARVED_MESH_GENERATOR_H_
#define _CARVED_MESH_GENERATOR_H_

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_

#include <CGAL/Polygon_mesh_processing/clip.h>

#include "PolygonMeshProcessing.h"
#include "Silhouette.h"
#include "calculateTau.h"

namespace LowpolyGen {

class CarvedMeshGenerator {
 private:
  const std::vector<Eigen::Vector3d>& _K;

 public:
  CarvedMeshGenerator(const std::vector<Eigen::Vector3d>& K);
  SurfaceMesh operator()(const SurfaceMesh& Mi, const SurfaceMesh& Mv, int N,
                         double epsilonTau);
};

};  // namespace LowpolyGen

#endif  // _CARVED_MESH_GENERATOR_H_