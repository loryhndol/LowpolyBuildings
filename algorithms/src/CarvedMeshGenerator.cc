#include "engine/CarvedMeshGenerator.h"

namespace LowpolyGen {
CarvedMeshGenerator::CarvedMeshGenerator(const std::vector<Eigen::Vector3d>& K,
                                         const Config& conf)
    : _K(K), _conf(conf) {}

SurfaceMesh CarvedMeshGenerator::run(const SurfaceMesh& Mi,
                                     const SurfaceMesh& Mv, int N,
                                     double epsilonTau) {
  using SurfaceMesh = CGAL::Surface_mesh<CGAL::Point_3<Kernel>>;

  // calculate bbox
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

  double diagonalLength = (maxCoords - minCoords).norm();

  std::vector<SurfaceMesh> PrimitiveSet(_K.size());
  // repairMesh(Mi);

  // #pragma omp parallel for
  for (int i = 0; i < _K.size(); i++) {
    if (i < _K.size() - 1) {
      printf("\rExtract Visual Hull for carving [Running] [%.2lf%%]:",
             (i + 1) * 100.0 / _K.size());
    } else {
      printf("\rExtract Visual Hull for carving [Finished] [%.2lf%%]:",
             (i + 1) * 100.0 / _K.size());
    }
    Eigen::Vector3d k = _K[i];
    CGAL::Vector_3<Kernel> direction{-k.x(), -k.y(), -k.z()};
    CGAL::Point_3<Kernel> origin{k.x(), k.y(), k.z()};

    Eigen::Vector3d cameraUp, cameraRight;

    SurfaceMesh newMi = Mi;

    Kernel kernel;
    typename Kernel::Plane_3 plane =
        kernel.construct_plane_3_object()(CGAL::ORIGIN, direction);

    // cut the mesh and get the positive part (keep the negative part by default
    // in CGAL)
    CGAL::Polygon_mesh_processing::clip(newMi, plane,
                                        CGAL::parameters::clip_volume(true));

    Silhouette S(newMi, origin, direction, diagonalLength);
    S.simplify();

    // caculate 2D bounding box of CCW_Loop;
    auto silhouetteArray = S.connectedLoops();
    CGAL::Bbox_2 boundingBox2D = silhouetteArray[0].bbox();

    double largestDiagonalLengthOfBBoxes = 0.0;
    double largestArea = 0.0;

    std::vector<CGAL::Polygon_with_holes_2<Kernel>> CCW_Loops;

    for (CGAL::Polygon_with_holes_2<Kernel>& P : silhouetteArray) {
      double diagonalLength = std::sqrt(std::pow(P.bbox().x_span(), 2) +
                                        std::pow(P.bbox().y_span(), 2));
      largestDiagonalLengthOfBBoxes =
          std::max(largestDiagonalLengthOfBBoxes, diagonalLength);
      largestArea =
          std::max(largestArea, P.bbox().x_span() * P.bbox().y_span());
      boundingBox2D += P.bbox();
    }

    // calculate extended bounding box
    double extendedXMin = boundingBox2D.xmin() - 0.2;
    double extendedXMax = boundingBox2D.xmax() + 0.2;
    double extendedYMin = boundingBox2D.ymin() - 0.2;
    double extendedYMax = boundingBox2D.ymax() + 0.2;

    // Reference : Boolean_set_operations_2/symmetric_difference.cpp
    // Construct extended bounded polygon with holes
    CGAL::Polygon_2<Kernel> boundary;
    boundary.push_back({extendedXMin, extendedYMin});
    boundary.push_back({extendedXMax, extendedYMin});
    boundary.push_back({extendedXMax, extendedYMax});
    boundary.push_back({extendedXMin, extendedYMax});

    std::vector<CGAL::Polygon_2<Kernel>> holesP;
    for (CGAL::Polygon_with_holes_2<Kernel> pwh : CCW_Loops) {
      CGAL::Polygon_2<Kernel> hole = pwh.outer_boundary();
      hole.reverse_orientation();
      holesP.push_back(hole);
    }

    CGAL::Polygon_with_holes_2<Kernel> complementPath(boundary, holesP.begin(),
                                                      holesP.end());
    Eigen::Vector3d eigD(CGAL::to_double(direction.x()),
                         CGAL::to_double(direction.y()),
                         CGAL::to_double(direction.z()));
    SurfaceMesh extrudedMesh =
        extrude(complementPath, eigD, 0.125 * diagonalLength);

    double dx = CGAL::to_double(k.x()) * 0.25 * diagonalLength;
    double dy = CGAL::to_double(k.y()) * 0.25 * diagonalLength;
    double dz = CGAL::to_double(k.z()) * 0.25 * diagonalLength;

    CGAL::Vector_3<Kernel> position(dx, dy, dz);

    CGAL::Aff_transformation_3<Kernel> aff(CGAL::Translation(), position);

    for (auto& p : extrudedMesh.points()) {
      p = aff.transform(p);
    }

    CGAL::IO::write_OBJ("tmp/carvePi.obj", extrudedMesh);
    PrimitiveSet[i] = extrudedMesh;
  }

  for (int i = 0; i < PrimitiveSet.size(); i++) {
    std::stringstream ss;
    ss << i;
    std::string idx;
    ss >> idx;
    CGAL::IO::write_OBJ("tmp/carvedP_" + idx + ".obj", PrimitiveSet[i]);
  }

  int n = 0;
  SurfaceMesh Mc = Mv;
  double tau = calculateTau(Mv, Mi, diagonalLength);
  while (n < _conf.N) {
    int bestIdx = -1;
    double deltaTauBest = 0;
    for (int i = 0; i < PrimitiveSet.size(); i++) {
      double tauP =
          calculateTau(subtract(Mc, PrimitiveSet[i]), Mi, diagonalLength);
      double deltaTauP = tau - tauP;
      if (deltaTauP > deltaTauBest) {
        deltaTauBest = deltaTauP;
        bestIdx = i;
      }
    }

    if (deltaTauBest >= _conf.epsilonTau) {
      Mc.clear();
      Mc = subtract(Mc, PrimitiveSet[bestIdx]);
      PrimitiveSet.erase(PrimitiveSet.begin() + bestIdx);
      n += 1;
      tau = tau - deltaTauBest;
    } else {
      break;
    }
  }

  return Mc;
}

}  // namespace LowpolyGen