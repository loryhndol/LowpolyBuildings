#include "engine/Silhouette.h"

namespace LowpolyGen {
/**
 * Adapted from The book `CGAL Arrangement and their applications`
 * Section 8.4 Application: Obtaining Silhouettes of Polyhedra.
 *
 * \param direction
 * The function will project the surfaceMesh mesh onto
 * the plane perpendicular to the direction
 *
 * \param Mi Source Surface_Mesh
 *
 * \return Polygon_set
 * The silhouette contains outer boundary and inner holes,
 * which represented as a surfaceMesh set
 *
 */
Silhouette::Silhouette(const SurfaceMesh& Mi, const Kernel::Point_3& origin,
                       Kernel::Vector_3& direction, double l) {
  Eigen::Vector3d d(CGAL::to_double(origin.x()), CGAL::to_double(origin.y()),
                    CGAL::to_double(origin.z()));
  Eigen::Vector3d cameraPosition = d;
  Eigen::Vector3d cameraOldPosition(0, 0, 3 * l);
  // camera up 定义为与旋转轴平行
  Eigen::Vector3d beforeRotation(0, 0, 1);
  Eigen::Vector3d cameraUp = cameraOldPosition.cross(cameraPosition);
  cameraUp.normalize();
  Eigen::Vector3d cameraRight = cameraPosition.cross(cameraUp);
  cameraRight.normalize();

  using Polygon = CGAL::Polygon_2<Kernel>;

  // Go over the surfaceMesh facets and project them onto the plane
  std::list<Polygon> polygons;

  typename Kernel::Compare_z_3 cmp_z{};
  typename Kernel::Construct_projected_xy_point_2 proj{};
  typename Kernel::Construct_translated_point_3 translate{};
  typename Kernel::Point_3 modelOrigin(CGAL::ORIGIN);
  // typename Kernel::Plane_3 plane = kernel.construct_plane_3_object()(origin,
  // direction);
  typename Kernel::Plane_3 plane(modelOrigin, direction);
  // typename Kernel::Point_3 r = translate(origin, direction.vector());
  typename Kernel::Point_3 r = translate(modelOrigin, direction);

  typename SurfaceMesh::Face_iterator fit;

  // https://www.cnblogs.com/nobodyzhou/p/6145030.html
  std::vector<int> shouldOutput(Mi.number_of_faces(), 0);
  std::vector<Polygon> tobeOutput(Mi.number_of_faces());

  for (int i = 0; i < Mi.number_of_faces(); i++) {
    typename SurfaceMesh::Face_iterator fit = Mi.faces_begin() + i;
    CGAL::Vector_3<Kernel> normal =
        CGAL::Polygon_mesh_processing::compute_face_normal(*fit, Mi);
    if (CGAL::angle(translate(modelOrigin, normal), modelOrigin, r) ==
        CGAL::RIGHT) {
      continue;
    }

    // Go over the facet vertices and project them

    Eigen::Vector3d n(CGAL::to_double(direction.x()),
                      CGAL::to_double(direction.y()),
                      CGAL::to_double(direction.z()));
    Eigen::Vector3d O(CGAL::to_double(modelOrigin.x()),
                      CGAL::to_double(modelOrigin.y()),
                      CGAL::to_double(modelOrigin.z()));

    const double constant1 = n.dot(O);
    const double constant2 = n.squaredNorm();

    Polygon polygon;

    for (typename SurfaceMesh::Vertex_index vd :
         Mi.vertices_around_face(Mi.halfedge(*fit))) {
      const CGAL::Point_3<Kernel>& point = Mi.point(vd);
      Eigen::Vector3d v(CGAL::to_double(point.x()), CGAL::to_double(point.y()),
                        CGAL::to_double(point.z()));

      double t = (constant1 - n.dot(v)) / constant2;

      Eigen::Vector3d projectedV = v + n * t;

      double upComponent = projectedV.dot(cameraUp);
      double rightComponent = projectedV.dot(cameraRight);

      polygon.push_back(CGAL::Point_2<Kernel>(rightComponent, upComponent));
    }

    if (CGAL::angle(translate(modelOrigin, normal), modelOrigin, r) ==
        CGAL::OBTUSE) {
      polygon.reverse_orientation();
    }

    if (polygon.is_simple() && polygon.size() >= 3) {
      tobeOutput[i] = polygon;
      shouldOutput[i] = 1;
    }
  }

  for (int i = 0; i < shouldOutput.size(); i++) {
    if (shouldOutput[i] != 0) {
      polygons.push_back(tobeOutput[i]);
    }
  }

  std::vector<CGAL::Polygon_with_holes_2<Kernel>> silhouetteArray;
  CGAL::join(polygons.begin(), polygons.end(),
             std::back_inserter(silhouetteArray));
  _cameraUp = cameraUp;
  _cameraRight = cameraRight;
  _silhouette2D = silhouetteArray;
}

void Silhouette::simplify() {
  namespace PS = CGAL::Polyline_simplification_2;
  using Vb = PS::Vertex_base_2<Kernel>;
  using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
  using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
  using CDT =
      CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS,
                                                 CGAL::Exact_predicates_tag>;
  using CDTplus = CGAL::Constrained_triangulation_plus_2<CDT>;

  using Constraint_iterator = typename CDTplus::Constraint_iterator;
  using Vertices_in_constraint_iterator =
      typename CDTplus::Vertices_in_constraint_iterator;
  using Points_in_constraint_iterator =
      typename CDTplus::Points_in_constraint_iterator;
  using Polygon_with_Holes = CGAL::Polygon_with_holes_2<Kernel>;

  // caculate 2D bounding box of CCW_Loop;
  CGAL::Bbox_2 boundingBox2D = _silhouette2D[0].bbox();

  std::vector<Polygon_with_Holes> result;

  double largestDiagonalLengthOfBBoxes = 0.0;
  double largestArea = 0.0;

  std::vector<Polygon_with_Holes> CCW_Loops;

  for (Polygon_with_Holes& P : _silhouette2D) {
    double diagonalLength = std::sqrt(std::pow(P.bbox().x_span(), 2) +
                                      std::pow(P.bbox().y_span(), 2));
    largestDiagonalLengthOfBBoxes =
        std::max(largestDiagonalLengthOfBBoxes, diagonalLength);
    largestArea = std::max(largestArea, P.bbox().x_span() * P.bbox().y_span());
    boundingBox2D += P.bbox();
  }

  for (Polygon_with_Holes& P : _silhouette2D) {
    CDTplus ct;
    // outer boundary simplification
    CGAL::Polygon_2<Kernel> poly = P.outer_boundary();

    ct.insert_constraint(poly);

    double epsilonD = 0.01 * largestDiagonalLengthOfBBoxes;

    PS::simplify(ct, PS::Squared_distance_cost(),
                 PS::Stop_above_cost_threshold(std::pow(epsilonD, 2)));

    CGAL::Polygon_2<Kernel> boundary;

    typename CDTplus::Constraint_iterator cit;
    for (cit = ct.constraints_begin(); cit != ct.constraints_end(); ++cit) {
      for (const CGAL::Point_2<Kernel>& pt : ct.points_in_constraint(*cit)) {
        boundary.push_back(pt);
      }
    }

    if (std::abs(CGAL::to_double(boundary.area()) < 0.01 * largestArea)) {
      continue;
    }

    CCW_Loops.push_back(Polygon_with_Holes{boundary});
  }

  std::vector<Polygon_with_Holes> CW_Loops;

  for (Polygon_with_Holes& P : _silhouette2D) {
    if (P.has_holes()) {
      // holes simplification
      typename Polygon_with_Holes::Hole_const_iterator hit;
      for (hit = P.holes_begin(); hit != P.holes_end(); ++hit) {
        CDTplus holeCt;
        const CGAL::Polygon_2<Kernel>& hole = *hit;
        holeCt.insert_constraint(hole);
        double epsilonD = 0.02 * largestDiagonalLengthOfBBoxes;

        PS::simplify(holeCt, PS::Squared_distance_cost(),
                     PS::Stop_above_cost_threshold(std::pow(epsilonD, 2)));

        CGAL::Polygon_2<Kernel> simplifiedHole;

        for (auto cit = holeCt.constraints_begin();
             cit != holeCt.constraints_end(); ++cit) {
          for (const CGAL::Point_2<Kernel>& pt :
               holeCt.points_in_constraint(*cit)) {
            simplifiedHole.push_back(pt);
          }
        }

        if (std::abs(CGAL::to_double(simplifiedHole.area())) <
            0.01 * largestArea) {
          continue;
        }

        CGAL::Polygon_2<Kernel> bboxPoly;
        bboxPoly.push_back({boundingBox2D.xmin(), boundingBox2D.ymin()});
        bboxPoly.push_back({boundingBox2D.xmax(), boundingBox2D.ymin()});
        bboxPoly.push_back({boundingBox2D.xmax(), boundingBox2D.ymax()});
        bboxPoly.push_back({boundingBox2D.xmin(), boundingBox2D.ymax()});

        CGAL::Polygon_with_holes_2<Kernel> subtraction(bboxPoly);
        subtraction.add_hole(simplifiedHole);

        CW_Loops.push_back(subtraction);
      }
    }
  }

  CGAL::Polygon_2<Kernel> ccwLoopSingle;

  for (auto& pwh : CCW_Loops) {
    for (auto pt : pwh.outer_boundary()) {
      ccwLoopSingle.push_back(pt);
    }
  }

  result.push_back(Polygon_with_Holes{ccwLoopSingle});

  for (auto& pwh : CW_Loops) {
    result.push_back(pwh);
  }

  _silhouette2D.clear();
  _silhouette2D = result;
}

std::vector<CGAL::Polygon_with_holes_2<Kernel>> Silhouette::connectedLoops() {
  return _silhouette2D;
}

}  // namespace LowpolyGen