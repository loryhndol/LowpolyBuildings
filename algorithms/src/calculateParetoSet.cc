#include "engine/calculateParetoSet.h"

namespace LowpolyGen {
std::vector<SurfaceMesh> calculateParetoSet(const SurfaceMesh& mesh,
                                            int maxTolerantElementsCnt) {
  auto MeshApprox =
      convertSurfaceMeshKernel<Kernel, CGAL::Simple_cartesian<double>>(mesh);

  // the following code operates on the precision
  // `CGAL::Simple_cartesian<double>`
  using KernelApprox = CGAL::Simple_cartesian<double>;
  using SurfaceMeshApprox = CGAL::Surface_mesh<KernelApprox::Point_3>;
  std::vector<SurfaceMeshApprox> meshArray;
  while (MeshApprox.number_of_faces() > 4) {
    // keep 90% of edges
    double stop_ratio = 0.9;
    CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<
        SurfaceMeshApprox>
        stop(stop_ratio);

    int r = CGAL::Surface_mesh_simplification::edge_collapse(MeshApprox, stop);

    // edge - flip if any pair of adjacent triangles has an obtuse dihedral
    // angle larger than or if the exterior dihedral angle is smaller than

    std::vector<SurfaceMeshApprox::Halfedge_index> toBeFlipped;

    SurfaceMeshApprox::halfedge_iterator hit;
    for (hit = MeshApprox.halfedges_begin(); hit != MeshApprox.halfedges_end();
         ++hit) {
      SurfaceMeshApprox::Halfedge_index oppositeHe = MeshApprox.opposite(*hit);

      // calculate dihedral angle with normals
      CGAL::Vector_3<KernelApprox> n1 =
          CGAL::Polygon_mesh_processing::compute_face_normal(
              MeshApprox.face(*hit), MeshApprox);
      CGAL::Vector_3<KernelApprox> n2 =
          CGAL::Polygon_mesh_processing::compute_face_normal(
              MeshApprox.face(oppositeHe), MeshApprox);

      double n1Length = std::sqrt(CGAL::to_double(n1.squared_length()));
      double n2Length = std::sqrt(CGAL::to_double(n2.squared_length()));
      double dihedralAngle =
          CGAL::to_double(CGAL::scalar_product(n1, n2)) / (n1Length * n2Length);

      double theta1 = 175.0 / 180.0 * M_PI;
      double theta2 = M_PI;

      if ((std::abs(dihedralAngle) > theta1 &&
           std::abs(dihedralAngle) < theta2)) {
        toBeFlipped.push_back(*hit);
      }
    }

    for (auto& he : toBeFlipped) {
      CGAL::Euler::flip_edge(he, MeshApprox);
    }

    if (MeshApprox.number_of_faces() < maxTolerantElementsCnt) {
      meshArray.push_back(MeshApprox);
    }

    if (r == 0) {
      break;
    }
  }

  // calculate the bounding box of Mi
  int idForPoint = 0;
  Eigen::Vector3d maxCoords;
  Eigen::Vector3d minCoords;
  for (const KernelApprox::Point_3& pt : MeshApprox.points()) {
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

  double l = (maxCoords - minCoords).norm();

  std::vector<SurfaceMeshApprox> paretoSet;
  std::vector<double> visualMesure;
  for (int i = 0; i < meshArray.size(); i++) {
    double tau = calculateTau(meshArray[i], mesh, l);
    visualMesure.push_back(std::exp(-tau));
  }

  int curIdx = 0;
  paretoSet.push_back(meshArray[0]);
  for (int i = 1; i < meshArray.size(); i++) {
    if (visualMesure[i] > visualMesure[curIdx]) {
      paretoSet.push_back(meshArray[i]);
      curIdx = i;
    }
  }

  std::vector<SurfaceMesh> retArray;
  for (auto& m : meshArray) {
    auto retMesh =
        convertSurfaceMeshKernel<CGAL::Simple_cartesian<double>, Kernel>(m);
    retArray.push_back(retMesh);
  }

  return retArray;
}

}  // namespace LowpolyGen