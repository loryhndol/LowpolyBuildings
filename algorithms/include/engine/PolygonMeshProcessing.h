#pragma once

#ifndef _COMMON_H_
#include "Common.h"
#endif
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Polygon_with_holes_2.h>

namespace LowpolyGen {

/**
 * extrude the 2D path along the 3D direction
 * \param vlen the length along the extrusion
 */
SurfaceMesh extrude(CGAL::Polygon_with_holes_2<Kernel>& path2D,
                    const Eigen::Vector3d& cameraPosition, double vlen);

SurfaceMesh intersect(const SurfaceMesh& A, const SurfaceMesh& B);

SurfaceMesh makeBBox(const SurfaceMesh& mesh);

bool WithinSilhouette(const Kernel::Point_2& pt,
                      const CGAL::Polygon_with_holes_2<Kernel>& silhouette);

}  // namespace LowpolyGen
