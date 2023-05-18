#ifndef _CARVED_MESH_GENERATOR_H_
#define _CARVED_MESH_GENERATOR_H_

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_

namespace LowpolyGen {

class CarvedMeshGenerator {
 public:
  CarvedMeshGenerator() = default;
  SurfaceMesh operator()(const SurfaceMesh& Mi, const SurfaceMesh& Mv, int N,
                         double epsilonTau);
};

};  // namespace LowpolyGen

#endif  // _CARVED_MESH_GENERATOR_H_