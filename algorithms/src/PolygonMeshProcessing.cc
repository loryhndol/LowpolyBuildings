#include "engine/PolygonMeshProcessing.h"

namespace LowpolyGen {

bool WithinSilhouette(const Kernel::Point_2& pt,
                      const CGAL::Polygon_with_holes_2<Kernel>& silhouette) {
  if (silhouette.outer_boundary().bounded_side(pt) == CGAL::ON_BOUNDED_SIDE) {
    return true;
  }
  for (auto hole = silhouette.holes_begin(); hole != silhouette.holes_end();
       ++hole) {
    if (hole->bounded_side(pt) == CGAL::ON_BOUNDED_SIDE) {
      return true;
    }
  }
  return false;
}

Kernel::Point_3 point2dTo3d(const Kernel::Point_2& pt2d,
                            const Eigen::Vector3d& cameraPosition,
                            const Eigen::Vector3d& cameraUp,
                            const Eigen::Vector3d& cameraRight,
                            const Eigen::Vector3d& d) {
  CGAL::Vector_3<Kernel> dir(d.x(), d.y(), d.z());
  CGAL::Point_3<Kernel> origin(0, 0, 0);
  CGAL::Plane_3<Kernel> plane(origin, dir);
  CGAL::Point_3<Kernel> pt3d(pt2d.x(), pt2d.y(), 0);
  CGAL::Vector_3<Kernel> cameraToPt3d(pt3d.x() - cameraPosition.x(),
                                      pt3d.y() - cameraPosition.y(),
                                      pt3d.z() - cameraPosition.z());
  double distance = CGAL::to_double(cameraToPt3d * dir) /
                    CGAL::to_double(dir.squared_length());
  CGAL::Point_3<Kernel> projection = pt3d - distance * dir;
  Eigen::Vector3d cameraToProjection(
      CGAL::to_double(projection.x()) - cameraPosition.x(),
      CGAL::to_double(projection.y()) - cameraPosition.y(),
      CGAL::to_double(projection.z()) - cameraPosition.z());
  double x = cameraToProjection.dot(cameraRight) / cameraRight.norm();
  double y = cameraToProjection.dot(cameraUp) / cameraUp.norm();
  double z = cameraToProjection.dot(d) / d.norm();
  return Kernel::Point_3(x, y, z);
}

SurfaceMesh extrude(CGAL::Polygon_with_holes_2<Kernel>& Path2D,
                    const Eigen::Vector3d& cameraPosition, double vlen) {
  Eigen::Vector3d cameraOldPosition(0, 0, 1);
  // camera up is parallel to the rotation axis
  Eigen::Vector3d beforeRotation(0, 0, 1);
  Eigen::Vector3d cameraUp = cameraOldPosition.cross(cameraPosition);
  cameraUp.normalize();
  Eigen::Vector3d cameraRight = cameraPosition.cross(cameraUp);
  cameraRight.normalize();

  using Polygon_with_Holes = CGAL::Polygon_with_holes_2<Kernel>;

  using CDT =
      CGAL::Constrained_Delaunay_triangulation_2<Kernel, CGAL::Default,
                                                 CGAL::Exact_intersections_tag>;
  using CDTplus = CGAL::Constrained_triangulation_plus_2<CDT>;

  CDTplus ct;

  ct.insert_constraint(Path2D.outer_boundary());
  typename Polygon_with_Holes::Hole_const_iterator hit;
  for (hit = Path2D.holes_begin(); hit != Path2D.holes_end(); ++hit) {
    ct.insert_constraint(*hit);
  }

  std::vector<CGAL::Point_2<Kernel>> SilhouetteBoundaryAndHoles;
  typename CDTplus::Constraint_iterator cit;
  typename CDTplus::Points_in_constraint_iterator pit;

  int numVerticesOuterBoundary;
  int numHoles;
  std::vector<int> numVerticesHolesArray;

  CGAL::Polygon_2<Kernel> bound = Path2D.outer_boundary();
  numVerticesOuterBoundary = bound.size();
  for (CGAL::Point_2<Kernel>& v : bound) {
    SilhouetteBoundaryAndHoles.push_back(v);
  }

  numHoles = Path2D.number_of_holes();

  for (CGAL::Polygon_2<Kernel>& hole : Path2D.holes()) {
    numVerticesHolesArray.push_back(hole.size());
    // hole.reverse_orientation();
    typename CGAL::Polygon_2<Kernel>::Vertex_iterator vit;

    for (vit = hole.vertices_begin(); vit != hole.vertices_end(); ++vit) {
      SilhouetteBoundaryAndHoles.push_back(*vit);
    }
  }

  SurfaceMesh mesh;

  typename CDTplus::Finite_faces_iterator ctfit;

  // save the face if its barycenter lies within the polygon
  for (ctfit = ct.finite_faces_begin(); ctfit != ct.finite_faces_end();
       ++ctfit) {
    CGAL::Vector_2<Kernel> sum(0, 0);

    for (int i = 0; i < 3; i++) {
      CGAL::Point_2<Kernel> pt = ctfit->vertex(i)->point();
      sum += CGAL::Vector_2<Kernel>(pt.x(), pt.y());
    }
    sum /= 3;

    if (WithinSilhouette(Kernel::Point_2(sum.x(), sum.y()), Path2D)) {
      std::vector<SurfaceMesh::Vertex_index> indexArray;
      for (int cnt = 0; cnt < 3; cnt++) {
        CGAL::Point_2<Kernel>& u = ctfit->vertex(cnt)->point();
        CGAL::Point_3<Kernel> pt = point2dTo3d(
            u, cameraPosition, cameraUp, cameraRight, Eigen::Vector3d(0, 0, 0));
        SurfaceMesh::Vertex_index index = mesh.add_vertex(pt);
        indexArray.push_back(index);
      }
      mesh.add_face(indexArray[0], indexArray[1], indexArray[2]);
    }
  }

  CGAL::IO::write_OBJ("tmp/mesh.obj", mesh);

  CGAL::Surface_mesh<CGAL::Point_3<Kernel>> outputMesh;
  std::vector<CGAL::Point_3<Kernel>> outputMeshVertices;
  std::vector<SurfaceMesh::Vertex_index> outputVerticeDescriptors;

  std::vector<CGAL::Point_3<Kernel>> boundary3DPoints;

  for (int i = 0; i < SilhouetteBoundaryAndHoles.size(); i++) {
    boundary3DPoints.push_back(
        point2dTo3d(SilhouetteBoundaryAndHoles[i], cameraPosition, cameraUp,
                    cameraRight, Eigen::Vector3d(0, 0, vlen)));
  }

  // generate top and bottom boundaries
  for (int i = 0; i < SilhouetteBoundaryAndHoles.size(); i++) {
    CGAL::Point_3<Kernel> pt = boundary3DPoints[i];
    CGAL::Vector_3<Kernel> dir(cameraPosition.x(), cameraPosition.y(),
                               cameraPosition.z());
    dir *= -2 * vlen;

    pt = pt.transform(
        CGAL::Aff_transformation_3<Kernel>(CGAL::Translation(), dir));
    outputMeshVertices.push_back(pt);
    SurfaceMesh::Vertex_index vd = outputMesh.add_vertex(pt);
    outputVerticeDescriptors.push_back(vd);
  }

  int totalVerticesSize = outputVerticeDescriptors.size();

  for (int i = 0; i < SilhouetteBoundaryAndHoles.size(); i++) {
    CGAL::Point_3<Kernel> pt = boundary3DPoints[i];
    CGAL::Vector_3<Kernel> dir(cameraPosition.x(), cameraPosition.y(),
                               cameraPosition.z());
    dir *= 2 * vlen;

    pt = pt.transform(
        CGAL::Aff_transformation_3<Kernel>(CGAL::Translation(), dir));
    outputMeshVertices.push_back(pt);
    SurfaceMesh::Vertex_index vd = outputMesh.add_vertex(pt);
    outputVerticeDescriptors.push_back(vd);
  }

  // add side faces
  /*
   * verts of outer boundary[0] -> verts of hole[0]
   *                            -> verts of hole[1]
   * verts of outer boundary[1] -> verts of hole[0]
   *                            -> verts of hole[1]
   *                            -> verts of hole[2]
   * verts of outer boundary[N] -> verts of hole[0]
   */
  int globalIdx = 0;
  int polygonSize = numVerticesOuterBoundary;
  for (int i = globalIdx; i < globalIdx + polygonSize; i++) {
    if (i == (globalIdx + polygonSize - 1)) {
      SurfaceMesh::Vertex_index& indexU1 =
          outputVerticeDescriptors[i + totalVerticesSize];
      SurfaceMesh::Vertex_index& indexV1 =
          outputVerticeDescriptors[globalIdx + totalVerticesSize];
      SurfaceMesh::Vertex_index& indexW1 = outputVerticeDescriptors[i];

      outputMesh.add_face(indexU1, indexV1, indexW1);

      SurfaceMesh::Vertex_index& indexU2 = outputVerticeDescriptors[globalIdx];
      SurfaceMesh::Vertex_index& indexV2 = outputVerticeDescriptors[i];
      SurfaceMesh::Vertex_index& indexW2 =
          outputVerticeDescriptors[globalIdx + totalVerticesSize];

      outputMesh.add_face(indexU2, indexV2, indexW2);
    } else {  // notice the direction of normal vector
      SurfaceMesh::Vertex_index& indexU1 =
          outputVerticeDescriptors[i + totalVerticesSize];
      SurfaceMesh::Vertex_index& indexV1 =
          outputVerticeDescriptors[i + totalVerticesSize + 1];
      SurfaceMesh::Vertex_index& indexW1 = outputVerticeDescriptors[i];

      outputMesh.add_face(indexU1, indexV1, indexW1);

      SurfaceMesh::Vertex_index& indexU2 = outputVerticeDescriptors[i + 1];
      SurfaceMesh::Vertex_index& indexV2 = outputVerticeDescriptors[i];
      SurfaceMesh::Vertex_index& indexW2 =
          outputVerticeDescriptors[i + totalVerticesSize + 1];

      outputMesh.add_face(indexU2, indexV2, indexW2);
    }
  }

  globalIdx += polygonSize;

  for (int i = 0; i < numHoles; i++) {  // for each hole of outer boundary
    int holeSize = numVerticesHolesArray[i];
    for (int j = globalIdx; j < globalIdx + holeSize; j++) {
      if (j == (globalIdx + holeSize - 1)) {
        SurfaceMesh::Vertex_index& indexU1 =
            outputVerticeDescriptors[j + totalVerticesSize];
        SurfaceMesh::Vertex_index& indexV1 =
            outputVerticeDescriptors[globalIdx + totalVerticesSize];
        SurfaceMesh::Vertex_index& indexW1 = outputVerticeDescriptors[j];

        outputMesh.add_face(indexU1, indexV1, indexW1);

        SurfaceMesh::Vertex_index& indexU2 =
            outputVerticeDescriptors[globalIdx];
        SurfaceMesh::Vertex_index& indexV2 = outputVerticeDescriptors[j];
        SurfaceMesh::Vertex_index& indexW2 =
            outputVerticeDescriptors[globalIdx + totalVerticesSize];

        outputMesh.add_face(indexU2, indexV2, indexW2);
      } else {
        SurfaceMesh::Vertex_index& indexU1 =
            outputVerticeDescriptors[j + totalVerticesSize];
        SurfaceMesh::Vertex_index& indexV1 =
            outputVerticeDescriptors[j + totalVerticesSize + 1];
        SurfaceMesh::Vertex_index& indexW1 = outputVerticeDescriptors[j];

        outputMesh.add_face(indexU1, indexV1, indexW1);

        SurfaceMesh::Vertex_index& indexU2 = outputVerticeDescriptors[j + 1];
        SurfaceMesh::Vertex_index& indexV2 = outputVerticeDescriptors[j];
        SurfaceMesh::Vertex_index& indexW2 =
            outputVerticeDescriptors[j + totalVerticesSize + 1];

        outputMesh.add_face(indexU2, indexV2, indexW2);
      }
    }

    // offset for former holes
    globalIdx += holeSize;
  }

  CGAL::IO::write_OBJ("tmp/side.obj", outputMesh);

  SurfaceMesh bottomMesh = mesh;  // backup for later use
  // add up faces
  CGAL::Vector_3<Kernel> topAffVec(cameraPosition.x(), cameraPosition.y(),
                                   cameraPosition.z());
  topAffVec *= -2 * vlen;

  CGAL::Aff_transformation_3<Kernel> topAff(CGAL::Translation(), topAffVec);

  CGAL::Polygon_mesh_processing::transform(topAff, mesh);

  for (const typename SurfaceMesh::Face_index& fd : mesh.faces()) {
    std::vector<SurfaceMesh::Vertex_index> extrudedVdArray;
    for (const SurfaceMesh::Vertex_index& vd :
         mesh.vertices_around_face(mesh.halfedge(fd))) {
      bool vdExisted = false;
      for (int i = 0; i < totalVerticesSize; i++) {
        SurfaceMesh::Vertex_index outputVd = outputVerticeDescriptors[i];
        if (CGAL::x_equal(mesh.point(vd), outputMesh.point(outputVd)) &&
            CGAL::y_equal(mesh.point(vd), outputMesh.point(outputVd)) &&
            CGAL::z_equal(mesh.point(vd), outputMesh.point(outputVd))) {
          extrudedVdArray.push_back(outputVd);
          vdExisted = true;
          break;
        }
      }
      if (!vdExisted) {
        SurfaceMesh::Vertex_index meshVd =
            outputMesh.add_vertex(mesh.point(vd));
        extrudedVdArray.push_back(meshVd);
      }
    }
    assert(extrudedVdArray.size() == 3);
    outputMesh.add_face(extrudedVdArray[0], extrudedVdArray[1],
                        extrudedVdArray[2]);
  }

  CGAL::IO::write_OBJ("tmp/outputMesh.obj", outputMesh);

  // add bottom mesh
  CGAL::Vector_3<Kernel> botAffVec(cameraPosition.x(), cameraPosition.y(),
                                   cameraPosition.z());
  botAffVec *= 2 * vlen;

  CGAL::Aff_transformation_3<Kernel> botAff(CGAL::Translation(), botAffVec);

  CGAL::Polygon_mesh_processing::transform(botAff, bottomMesh);

  for (typename SurfaceMesh::Face_index fd : bottomMesh.faces()) {
    std::vector<SurfaceMesh::Vertex_index> extrudedVdArray;
    for (const SurfaceMesh::Vertex_index& vd :
         bottomMesh.vertices_around_face(bottomMesh.halfedge(fd))) {
      bool vdExisted = false;
      for (int i = totalVerticesSize; i < outputVerticeDescriptors.size();
           i++) {
        SurfaceMesh::Vertex_index outputVd = outputVerticeDescriptors[i];
        if (CGAL::x_equal(bottomMesh.point(vd), outputMesh.point(outputVd)) &&
            CGAL::y_equal(bottomMesh.point(vd), outputMesh.point(outputVd)) &&
            CGAL::z_equal(bottomMesh.point(vd), outputMesh.point(outputVd))) {
          extrudedVdArray.push_back(outputVd);
          vdExisted = true;
          break;
        }
      }
      if (!vdExisted) {
        SurfaceMesh::Vertex_index extrudedVd =
            outputMesh.add_vertex(bottomMesh.point(vd));
        extrudedVdArray.push_back(extrudedVd);
      }
    }
    // the normal direction at the bottom should be opposite to the top
    outputMesh.add_face(extrudedVdArray[0], extrudedVdArray[2],
                        extrudedVdArray[1]);
  }
  repairMesh(outputMesh);
  CGAL::IO::write_OBJ("tmp/output2.obj", outputMesh);
  return outputMesh;
}

SurfaceMesh intersect(const SurfaceMesh& A, const SurfaceMesh& B) {
  if (CGAL::Polygon_mesh_processing::does_self_intersect(A)) {
    std::cout << "mesh A is self intersecting." << std::endl;
  }
  if (CGAL::Polygon_mesh_processing::does_self_intersect(B)) {
    std::cout << "mesh B is self intersecting." << std::endl;
  }
  SurfaceMesh cut = A;
  SurfaceMesh target = B;
  SurfaceMesh result;
  CGAL::Polygon_mesh_processing::corefine_and_compute_intersection(cut, target,
                                                                   result);
  return result;
}

SurfaceMesh subtract(const SurfaceMesh& A, const SurfaceMesh& B) {
  if (CGAL::Polygon_mesh_processing::does_self_intersect(A)) {
    std::cout << "mesh A is self intersecting." << std::endl;
  }
  if (CGAL::Polygon_mesh_processing::does_self_intersect(B)) {
    std::cout << "mesh B is self intersecting." << std::endl;
  }
  SurfaceMesh cut = A;
  SurfaceMesh target = B;
  SurfaceMesh result;
  CGAL::Polygon_mesh_processing::corefine_and_compute_difference(cut, target,
                                                                 result);
  return result;
}

SurfaceMesh makeBBox(const SurfaceMesh& Mi) {
  using vertex_descriptor = typename SurfaceMesh::Vertex_index;

  // calculate the bounding box of Mi
  int idForPoint = 0;
  Eigen::Vector3d maxCoords;
  Eigen::Vector3d minCoords;
  for (const Kernel::Point_3& pt : Mi.points()) {
    if (idForPoint == 0) {
      for (int d = 0; d < 3; d++) {
        minCoords[d] = maxCoords[d] = CGAL::to_double(pt.cartesian(d));
      }
    } else {
      for (int d = 0; d < 3; d++) {
        double res = CGAL::to_double(pt.cartesian(d));
        maxCoords[d] = std::max(maxCoords[d], res);
        minCoords[d] = std::min(minCoords[d], res);
      }
    }
    idForPoint += 1;
  }

  SurfaceMesh Mv;

  // Add the points as vertices
  vertex_descriptor vertexIndexArray[8];

  vertexIndexArray[0] =
      Mv.add_vertex(Kernel::Point_3(minCoords[0], minCoords[1], minCoords[2]));
  vertexIndexArray[1] =
      Mv.add_vertex(Kernel::Point_3(maxCoords[0], minCoords[1], minCoords[2]));
  vertexIndexArray[2] =
      Mv.add_vertex(Kernel::Point_3(maxCoords[0], maxCoords[1], minCoords[2]));
  vertexIndexArray[3] =
      Mv.add_vertex(Kernel::Point_3(minCoords[0], maxCoords[1], minCoords[2]));
  vertexIndexArray[4] =
      Mv.add_vertex(Kernel::Point_3(minCoords[0], minCoords[1], maxCoords[2]));
  vertexIndexArray[5] =
      Mv.add_vertex(Kernel::Point_3(maxCoords[0], minCoords[1], maxCoords[2]));
  vertexIndexArray[6] =
      Mv.add_vertex(Kernel::Point_3(maxCoords[0], maxCoords[1], maxCoords[2]));
  vertexIndexArray[7] =
      Mv.add_vertex(Kernel::Point_3(minCoords[0], maxCoords[1], maxCoords[2]));

  // right-hand side rule to represent outside
  // bottom face
  Mv.add_face(vertexIndexArray[0], vertexIndexArray[3], vertexIndexArray[2]);
  Mv.add_face(vertexIndexArray[2], vertexIndexArray[1], vertexIndexArray[0]);

  // top face
  Mv.add_face(vertexIndexArray[4], vertexIndexArray[5], vertexIndexArray[6]);
  Mv.add_face(vertexIndexArray[6], vertexIndexArray[7], vertexIndexArray[4]);

  // back face
  Mv.add_face(vertexIndexArray[7], vertexIndexArray[3], vertexIndexArray[0]);
  Mv.add_face(vertexIndexArray[0], vertexIndexArray[4], vertexIndexArray[7]);

  // front face
  Mv.add_face(vertexIndexArray[5], vertexIndexArray[1], vertexIndexArray[2]);
  Mv.add_face(vertexIndexArray[2], vertexIndexArray[6], vertexIndexArray[5]);

  // left face
  Mv.add_face(vertexIndexArray[4], vertexIndexArray[0], vertexIndexArray[1]);
  Mv.add_face(vertexIndexArray[1], vertexIndexArray[5], vertexIndexArray[4]);

  // right face
  Mv.add_face(vertexIndexArray[6], vertexIndexArray[2], vertexIndexArray[3]);
  Mv.add_face(vertexIndexArray[3], vertexIndexArray[7], vertexIndexArray[6]);

  return Mv;
}

void repairMesh(SurfaceMesh& mesh) {
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
    mesh.remove_face(f1);
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
}  // namespace LowpolyGen
