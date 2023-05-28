#pragma once

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_
#include <CGAL/Arrangement_2.h>
#include <CGAL/Boolean_set_operations_2.h>

// Polyhedron mesh
#include <CGAL/IO/Gps_iostream.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Surface_mesh.h>

#include <list>

namespace LowpolyGen {
/**
 * Generate a silhouette along a given direction
 */
class Silhouette {
 private:
  std::vector<CGAL::Polygon_with_holes_2<Kernel>> _silhouette2D;
  Eigen::Vector3d _cameraUp;
  Eigen::Vector3d _cameraRight;

 public:
  /**
   * Adapted from The book `CGAL Arrangement and their applications`
   * Section 8.4 Application: Obtaining Silhouettes of Polyhedra.
   *
   * \param direction
   * The function will project the surfaceMesh mesh onto
   * the plane perpendicular to the direction
   *
   */
  Silhouette(const SurfaceMesh& Mi, const Kernel::Point_3& origin,
             Kernel::Vector_3& direction, double l);
  void simplify();
  std::vector<CGAL::Polygon_with_holes_2<Kernel>> connectedLoops();
};

}  // namespace LowpolyGen
