#pragma once

#include <CGAL/Polygon_mesh_processing/internal/repair_extra.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh.h>

#include <vector>

// https://github.com/CGAL/cgal/issues/6608

template <class Kernel>
void repairMesh(CGAL::Surface_mesh<CGAL::Point_3<Kernel>>& mesh) {
  using SurfaceMesh = CGAL::Surface_mesh<CGAL::Point_3<Kernel>>;
  using halfedge_descriptor = typename SurfaceMesh::Halfedge_index;
  using face_descriptor = typename SurfaceMesh::Face_index;
  using vertex_descriptor = typename SurfaceMesh::Vertex_index;
  CGAL::Polygon_mesh_processing::remove_degenerate_edges(mesh);
  CGAL::Polygon_mesh_processing::remove_degenerate_faces(mesh);
  std::vector<std::pair<halfedge_descriptor, halfedge_descriptor>>
      halfedges_to_stitch;
  CGAL::Polygon_mesh_processing::collect_close_stitchable_boundary_edges(
      mesh, 1e-9, get(boost::vertex_point, mesh), halfedges_to_stitch);
  // std::cout << "halfedges to stich: " << halfedges_to_stitch.size() <<
  // std::endl;
  CGAL::Polygon_mesh_processing::stitch_borders(mesh, halfedges_to_stitch);

  std::vector<std::pair<face_descriptor, face_descriptor>> selfIntersections;

  CGAL::Polygon_mesh_processing::self_intersections(
      mesh, std::back_inserter(selfIntersections));
  for (auto& facePair : selfIntersections) {
    face_descriptor f1 = facePair.first;
    // face_descriptor f2 = facePair.second;
    mesh.remove_face(f1);
    // mesh.remove_face(f2);
  }

  std::vector<halfedge_descriptor> nonManifoldHalfedges;
  CGAL::Polygon_mesh_processing::non_manifold_vertices(
      mesh, std::back_inserter(nonManifoldHalfedges));
  for (auto& he : nonManifoldHalfedges) {
    mesh.remove_vertex(mesh.target(he));
  }

  CGAL::Polygon_mesh_processing::remove_degenerate_edges(mesh);
  CGAL::Polygon_mesh_processing::remove_degenerate_faces(mesh);
  CGAL::Polygon_mesh_processing::remove_isolated_vertices(mesh);

  typedef std::vector<std::size_t> CGAL_Polygon;
  std::vector<CGAL::Point_3<Kernel>> points;
  std::vector<CGAL_Polygon> polygons;
  CGAL::Polygon_mesh_processing::polygon_mesh_to_polygon_soup(mesh, points,
                                                              polygons);

  CGAL::Polygon_mesh_processing::repair_polygon_soup(
      points, polygons,
      CGAL::parameters::erase_all_duplicates(true).require_same_orientation(
          true));
  mesh.clear();
  // CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons,
                                                              mesh);

  std::vector<halfedge_descriptor> border_cycles;
  CGAL::Polygon_mesh_processing::extract_boundary_cycles(
      mesh, std::back_inserter(border_cycles));
  for (halfedge_descriptor h : border_cycles) {
    std::vector<face_descriptor> patch_facets;
    CGAL::Polygon_mesh_processing::triangulate_hole(
        mesh, h, std::back_inserter(patch_facets));
  }

  CGAL::Polygon_mesh_processing::remove_connected_components_of_negligible_size(
      mesh);

  CGAL::Polygon_mesh_processing::experimental::remove_self_intersections(
      mesh, CGAL::parameters::preserve_genus(false));
}