#ifndef _CARVED_MESH_GENERATOR_H_
#define _CARVED_MESH_GENERATOR_H_

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_

#include <CGAL/Polygon_mesh_processing/clip.h>

#include "PolygonMeshProcessing.h"
#include "Silhouette.h"
#include "calculateTau.hpp"

namespace LowpolyGen {

class CarvedMeshGenerator {
 private:
  const std::vector<Eigen::Vector3d>& _K;
  const Config& _conf;

 public:
  CarvedMeshGenerator(const std::vector<Eigen::Vector3d>& K,
                      const Config& conf);
  SurfaceMesh run(const SurfaceMesh& Mi, const SurfaceMesh& Mv, int N,
                  double epsilonTau);
};

};  // namespace LowpolyGen

#endif  // _CARVED_MESH_GENERATOR_H_