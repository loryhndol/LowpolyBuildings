#include "engine/Silhouette.h"

namespace LowpolyGen {
Silhouette::Silhouette(const SurfaceMesh& Mi, Kernel::Vector_3& direction,
                       double diagonalLength) {
  Kernel::Compare_z_3 cmp_z{};
  Kernel::Construct_projected_xy_point_2 proj{};
  typename Kernel::Construct_translated_point_3 translate{};
  typename Kernel::Point_3 modelOrigin(CGAL::ORIGIN);
  typename Kernel::Plane_3 plane(modelOrigin, direction);
  typename Kernel::Point_3 r = translate(modelOrigin, direction);

  auto fnormals =
      Mi.property_map<SurfaceMesh::Face_index, Kernel::Vector_3>("f:normals")
          .first;

  std::list<CGAL::Polygon_2<Kernel>> polygonList;
  for (SurfaceMesh::Face_index fd : Mi.faces()) {
    Kernel::Vector_3 normal = fnormals[fd];
    if (CGAL::angle(translate(modelOrigin, normal), modelOrigin, r) ==
        CGAL::RIGHT) {
      continue;
    }

    CGAL::Polygon_2<Kernel> polygon;
    // Go over the facet vertices and project them
    for (SurfaceMesh::Vertex_index vd :
         Mi.vertices_around_face(Mi.halfedge(fd))) {
      polygon.push_back(proj(plane, Mi.point(vd)));
    }

    if (CGAL::angle(translate(modelOrigin, normal), modelOrigin, r) ==
        CGAL::OBTUSE) {
      polygon.reverse_orientation();
    }

    if (polygon.is_simple() && polygon.size() >= 3) {
      polygonList.push_back(polygon);
    }
  }

  // for (auto poly : polygonList) {
  //   std::cout << "M ";
  //   for (auto pt : poly.vertices()) {
  //     std::cout << pt.x() << " " << pt.y() << " ";
  //   }
  std::cout << std::endl;
  CGAL::join(polygonList.begin(), polygonList.end(),
             std::back_inserter(_silhouette2D));
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