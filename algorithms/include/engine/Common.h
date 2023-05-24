#ifndef _COMMON_H_
#define _COMMON_H_
#define _USE_MATH_DEFINES
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace LowpolyGen {
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using SurfaceMesh = CGAL::Surface_mesh<CGAL::Point_3<Kernel>>;

struct __declspec(dllexport) Config {
  int N;
  int T;
  int k;
  double epsilon;
  double epsilonTau;
  double beta;
};

}  // namespace LowpolyGen

#endif  // _COMMON_H_