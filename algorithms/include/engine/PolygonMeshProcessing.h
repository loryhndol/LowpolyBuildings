#pragma once

#ifndef _COMMON_H_
#include "Common.h"
#endif
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Polygon_with_holes_2.h>

// repair mesh
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/internal/repair_extra.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

namespace LowpolyGen {

/**
 * extrude the 2D path along the 3D direction
 * \param vlen the length along the extrusion
 */
SurfaceMesh extrude(CGAL::Polygon_with_holes_2<Kernel>& path2D,
                    const Eigen::Vector3d& cameraPosition, double vlen);

SurfaceMesh intersect(const SurfaceMesh& A, const SurfaceMesh& B);

SurfaceMesh subtract(const SurfaceMesh& A, const SurfaceMesh& B);

SurfaceMesh makeBBox(const SurfaceMesh& Mi);

int dcmp(double x);

bool InPolygon(const CGAL::Point_2<Kernel>& P,
               const CGAL::Polygon_2<Kernel>& poly);

bool WithinSilhouette(const Kernel::Point_2& pt,
                      const CGAL::Polygon_with_holes_2<Kernel>& silhouette);

void repairMesh(SurfaceMesh& mesh);

#include <CGAL/boost/graph/copy_face_graph.h>

// Reference:
// https://doc.cgal.org/latest/BGL/BGL_polyhedron_3_2copy_polyhedron_8cpp-example.html#a5
template <class SourceKernel, class TargetKernel>
CGAL::Surface_mesh<CGAL::Point_3<TargetKernel>> convertSurfaceMeshKernel(
    const CGAL::Surface_mesh<CGAL::Point_3<SourceKernel>>& src) {
  CGAL::Surface_mesh<CGAL::Point_3<TargetKernel>> tar;
  CGAL::copy_face_graph(src, tar);
  return tar;
}

}  // namespace LowpolyGen
