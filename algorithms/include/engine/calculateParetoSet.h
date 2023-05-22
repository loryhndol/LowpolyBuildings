#pragma once

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>

#include "PolygonMeshProcessing.h"
#include "calculateTau.hpp"
#include "renderer.h"

namespace LowpolyGen {
std::vector<SurfaceMesh> calculateParetoSet(const SurfaceMesh& mesh,
                                            int maxTolerantElementsCnt);
}  // namespace LowpolyGen
