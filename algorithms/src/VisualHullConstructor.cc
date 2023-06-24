#include "engine/VisualHullConstructor.h"

namespace LowpolyGen {

VisualHullConstructor::VisualHullConstructor(const Config& conf)
    : _conf(conf){};

SurfaceMesh VisualHullConstructor::run(SurfaceMesh& Mi) {
  std::vector<SurfaceMesh> P;
  _topKDirections = pickTopViewDirections(_conf.k, Mi);

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

#pragma omp parallel for
  for (int id = 0; id < _topKDirections.size(); id++) {
    Eigen::Vector3d& d = _topKDirections[id];
    Kernel::Vector_3 direction(-d.x(), -d.y(), -d.z());
    Kernel::Point_3 origin{d.x() * 3 * diagonalLength,
                           d.y() * 3 * diagonalLength,
                           d.z() * 3 * diagonalLength};
    Silhouette S(Mi, origin, direction, diagonalLength);
    S.simplify();
    for (auto& L : S.connectedLoops()) {
      P.push_back(extrude(L, d, diagonalLength));
    }
  }

  SurfaceMesh Mv = makeBBox(Mi);
  double tau = calculateTau(Mv, Mi, diagonalLength);
  for (int n = 0; n < _conf.N; n++) {
    std::cout << "\r";
    std::cout << "[visual hull construction] iteration: " << n << "/"
              << _conf.N;
    if (n == _conf.N - 1) {
      std::cout << std::endl;
    }
    int bestIdx = 0;
    double deltaTauBest = 0;
    for (int i = 0; i < P.size(); i++) {
      double tauP = calculateTau(intersect(Mv, P[i]), Mi, diagonalLength);
      double deltaTauP = tau - tauP;
      if (deltaTauP > deltaTauBest) {
        deltaTauBest = deltaTauP;
        bestIdx = i;
      }
    }

    if (deltaTauBest >= _conf.epsilonTau) {
      SurfaceMesh newMv = intersect(Mv, P[bestIdx]);
      Mv.clear();
      Mv = newMv;
      P.erase(P.begin() + bestIdx);
      n += 1;
      tau = tau - deltaTauBest;
    } else {
      break;
    }
  }

  return Mv;
}

std::unordered_map<int, std::vector<int>>
VisualHullConstructor::groupTrianglesIntoRegions(SurfaceMesh& Mi) {
  std::vector<int> connectedComponents(Mi.number_of_faces());

  for (int i = 0; i < connectedComponents.size(); i++) {
    connectedComponents[i] = i;
  }

  auto fnormals =
      Mi.add_property_map<SurfaceMesh::Face_index, Kernel::Vector_3>(
            "f:normals", CGAL::NULL_VECTOR)
          .first;
  auto vnormals =
      Mi.add_property_map<SurfaceMesh::Vertex_index, Kernel::Vector_3>(
            "v:normals", CGAL::NULL_VECTOR)
          .first;

  CGAL::Polygon_mesh_processing::compute_normals(Mi, vnormals, fnormals);

  Graph disjointSet(Mi.number_of_faces());

#pragma omp parallel for
  for (size_t i = 0; i < Mi.number_of_faces(); i++) {
    // if (i == Mi.number_of_faces() - 1) {
    //   printf("\rmerging planes %.2lf\n",
    //          double(i) / double(Mi.number_of_faces()) * 100.0);
    // } else {
    //   printf("\rmerging planes %.2lf",
    //          double(i) / double(Mi.number_of_faces()) * 100.0);
    // }

    for (size_t j = i + 1; j < Mi.number_of_faces(); j++) {
      SurfaceMesh::Face_iterator fit1 = Mi.faces_begin() + i;
      SurfaceMesh::Face_iterator fit2 = Mi.faces_begin() + j;
      auto n1 = fnormals[*fit1];
      auto n2 = fnormals[*fit2];
      double prodN1N2 = CGAL::to_double(CGAL::scalar_product(n1, n2));
      double prodN1 = CGAL::to_double(CGAL::scalar_product(n1, n1));
      double prodN2 = CGAL::to_double(CGAL::scalar_product(n2, n2));
      double cosAngle = prodN1N2 / (std::sqrt(prodN1) * std::sqrt(prodN2));
      if (std::abs(cosAngle) >= std::cos(_conf.beta)) {
        disjointSet.unite(i, j);
      }
    }
  }

  std::unordered_map<int, std::vector<int>> regions;
  // initilaization, insert root node of each region
  for (int faceId = 0; faceId < disjointSet.numOfNodes(); faceId++) {
    if (disjointSet.find(faceId) == faceId) {
      std::vector<int> t;
      t.push_back(faceId);
      regions.insert(std::make_pair(faceId, t));
    }
  }

  // add faces belong to the same region
  for (int faceId = 0; faceId < static_cast<int>(Mi.number_of_faces());
       faceId++) {
    if (disjointSet.find(faceId) != faceId) {
      regions[disjointSet.find(faceId)].push_back(faceId);
    }
  }

  std::cout << "regions: " << regions.size() << std::endl;

  return std::move(regions);
}

bool EigenTupleLess(const std::tuple<double, Eigen::MatrixXd>& a,
                    const std::tuple<double, Eigen::MatrixXd>& b) {
  return std::get<0>(a) < std::get<0>(b);
}

void EigenVectorOfSmallestEigenValue(Eigen::VectorXd& eigenValues,
                                     Eigen::MatrixXd& eigenVectors) {
  std::vector<std::tuple<double, Eigen::MatrixXd>> eigenValueAndVector;
  int size = eigenValues.size();

  eigenValueAndVector.reserve(size);
  for (int i = 0; i < size; i++) {
    eigenValueAndVector.push_back({eigenValues[i], eigenVectors.col(i)});
  }

  std::sort(eigenValueAndVector.begin(), eigenValueAndVector.end(),
            EigenTupleLess);

  // refresh data
  for (int i = 0; i < size; i++) {
    eigenValues[i] = std::get<0>(eigenValueAndVector[i]);
    eigenVectors.col(i).swap(std::get<1>(eigenValueAndVector[i]));
  }
}

std::vector<WeightedVector> VisualHullConstructor::fitPlanesFromRegions(
    std::vector<int>& regionHead,
    std::unordered_map<int, std::vector<int>>& regions, const SurfaceMesh& Mi) {
  std::vector<WeightedVector> weightedViewDirections;

  for (auto areaRoot : regionHead) {
    Eigen::Vector3d X(0.0, 0.0, 0.0);

    double sumOfArea = 0;
    std::vector<double> areaOfTriangles;
    // the eigenvector corresponding to the smallest eigenvalue of the
    // following matrix
    Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(3, 3);

    std::vector<int> faceIndexArray = regions.at(areaRoot);

    for (int j = 0; j < faceIndexArray.size(); j++) {
      typename SurfaceMesh::Face_index fid =
          *(Mi.faces_begin() + faceIndexArray[j]);
      typename SurfaceMesh::Halfedge_index he = Mi.halfedge(fid);

      typename SurfaceMesh::Vertex_index vd0 = Mi.source(he);
      typename SurfaceMesh::Vertex_index vd1 = Mi.target(he);
      typename SurfaceMesh::Vertex_index vd2 = Mi.target(Mi.next(he));

      CGAL::Point_3<Kernel> v0 = Mi.point(vd0);
      CGAL::Point_3<Kernel> v1 = Mi.point(vd1);
      CGAL::Point_3<Kernel> v2 = Mi.point(vd2);

      CGAL::Triangle_3<Kernel> t(v0, v1, v2);
      Eigen::MatrixXd localMatrix;

      double area = std::sqrt(CGAL::to_double(t.squared_area()));
      areaOfTriangles.push_back(area);

      CGAL::Point_3<Kernel> barycenter = CGAL::barycenter(v0, 1, v1, 1, v2, 1);
      double bx = CGAL::to_double(barycenter.x());
      double by = CGAL::to_double(barycenter.y());
      double bz = CGAL::to_double(barycenter.z());

      Eigen::Vector3d tmpBarycenter;
      tmpBarycenter << bx, by, bz;

      X += area * tmpBarycenter;

      Eigen::MatrixXd Mi = Eigen::MatrixXd::Zero(3, 3);
      CGAL::Vector_3<Kernel> AB = t.vertex(2) - t.vertex(1);
      CGAL::Vector_3<Kernel> AC = t.vertex(3) - t.vertex(1);
      Eigen::Vector3d tmpAB;
      tmpAB << CGAL::to_double(AB.x()), CGAL::to_double(AB.y()),
          CGAL::to_double(AB.z());

      Eigen::Vector3d tmpAC;
      tmpAC << CGAL::to_double(AC.x()), CGAL::to_double(AC.y()),
          CGAL::to_double(AC.z());

      Mi.row(0) = tmpAB;
      Mi.row(1) = tmpAC;

      Eigen::MatrixXd tmp(3, 3);

      tmp << 10, 7, 0, 7, 10, 0, 0, 0, 0;

      localMatrix =
          2 * areaOfTriangles[j] / 72 * Mi * tmp * Mi.transpose() +
          areaOfTriangles[j] * tmpBarycenter * tmpBarycenter.transpose();

      lhs += localMatrix;

      for (int j = 0; j < areaOfTriangles.size(); j++) {
        sumOfArea += areaOfTriangles[j];
      }
      if (sumOfArea < 1e-15) {
        continue;
      }
    }
    X /= sumOfArea;

    Eigen::MatrixXd rhs = sumOfArea * X * X.transpose();

    Eigen::MatrixXd result = lhs - rhs;

    Eigen::EigenSolver<Eigen::MatrixXd> es(result);

    Eigen::VectorXd eigenValues = es.pseudoEigenvalueMatrix().diagonal();
    Eigen::MatrixXd eigenVectors = es.pseudoEigenvectors();
    EigenVectorOfSmallestEigenValue(eigenValues, eigenVectors);

    Eigen::Vector3d N = eigenVectors.col(0);
    WeightedVector dir(N, sumOfArea);
    weightedViewDirections.push_back(dir);
  }
  return weightedViewDirections;
}

std::vector<Eigen::Vector3d> VisualHullConstructor::generateViewDirections(
    std::vector<WeightedVector>& weightedViewDirections, int k) {
  std::vector<int> containsResult(
      weightedViewDirections.size() * (weightedViewDirections.size() - 1) / 2,
      0);
  std::vector<WeightedVector> tmpDWithWeight(
      weightedViewDirections.size() * (weightedViewDirections.size() - 1) / 2);
  std::vector<WeightedVector> DWithWeight;
  // for each pair of planes in K,
  // the cross product of their normal directions would result a direction
  //	parallel to both planes, resulting in the view direction set D
#pragma omp parallel for
  for (int i = 0; i < weightedViewDirections.size(); i++) {
    for (int j = i + 1; j < weightedViewDirections.size(); j++) {
      Eigen::Vector3d& n1 = weightedViewDirections[i]._d;
      Eigen::Vector3d& n2 = weightedViewDirections[j]._d;
      // special cases
      double dx = abs(n1.x() - n2.x());
      double dy = abs(n1.y() - n2.y());
      double dz = abs(n1.z() - n2.z());
      if (dx < _conf.epsilon && dy < _conf.epsilon && dz < _conf.epsilon) {
        continue;
      }
      if (std::abs(n1[0]) < _conf.epsilon && std::abs(n1[1]) < _conf.epsilon &&
          std::abs(n1[2]) < _conf.epsilon) {
        continue;
      }
      if (std::abs(n2[0]) < _conf.epsilon && std::abs(n2[1]) < _conf.epsilon &&
          std::abs(n2[2]) < _conf.epsilon) {
        continue;
      }
      WeightedVector vi(n1.cross(n2), weightedViewDirections[i]._weight +
                                          weightedViewDirections[j]._weight);
      int rowOffset;
      if (i == 0) {
        rowOffset = 0;
      } else if (i == 1) {
        rowOffset = weightedViewDirections.size() - 1;
      } else {
        rowOffset = (weightedViewDirections.size() - 1 +
                     weightedViewDirections.size() - 1 - (i - 1)) *
                    i / 2;
      }
      int colOffset = j - (i + 1);
      tmpDWithWeight[rowOffset + colOffset] = vi;
      containsResult[rowOffset + colOffset] = 1;
    }
  }

  for (int i = 0; i < containsResult.size(); i++) {
    if (containsResult[i] != 0) {
      DWithWeight.push_back(tmpDWithWeight[i]);
    }
  }
  std::cout << "Direction Set: " << DWithWeight.size() << std::endl;

  // remove duplicate, 1 for valid
  std::vector<int> duplicatedArray(DWithWeight.size(), 1);

#pragma omp parallel for
  for (int i = 0; i < DWithWeight.size(); i++) {
    for (int j = i + 1; j < DWithWeight.size(); j++) {
      if (DWithWeight[i]._d == DWithWeight[j]._d) {
        duplicatedArray[i] = 0;
        continue;
      }
      double dotProduct = DWithWeight[i]._d.dot(DWithWeight[j]._d);

      double cosAngle = std::abs(dotProduct) / DWithWeight[i]._d.norm();
      cosAngle /= DWithWeight[j]._d.norm();
      if (cosAngle >= std::cos(_conf.beta)) {
        duplicatedArray[i] = 0;
      }
    }
  }

  std::vector<WeightedVector> NoDuplication;
  for (int i = 0; i < duplicatedArray.size(); i++) {
    if (duplicatedArray[i] != 0) {
      NoDuplication.push_back(DWithWeight[i]);
    }
  }

  // Finally, we associate a weight with each view direction in D, which
  // equals to the sum of areas of the two planar regions. Therefore, we sort
  // the view directions by their weightsand pick the top k directions as the
  // final direction set.
  std::priority_queue<WeightedVector, std::vector<WeightedVector>,
                      std::greater<>>
      pqForWeightedViewDirection;

  std::cout << "Direction Set No Duplication: " << NoDuplication.size()
            << std::endl;
  for (int i = 0; i < NoDuplication.size(); i++) {
    pqForWeightedViewDirection.push(NoDuplication[i]);
  }

  std::vector<Eigen::Vector3d> finalViewDirections;

  int cnt = 0;
  while (!pqForWeightedViewDirection.empty()) {
    const WeightedVector& node = pqForWeightedViewDirection.top();
    pqForWeightedViewDirection.pop();
    if (cnt != k) {
      finalViewDirections.push_back(node._d);
      cnt++;
    } else {
      break;
    }
  }

  std::cout << "Total view directions: " << finalViewDirections.size()
            << std::endl;
  return finalViewDirections;
}

/**
 * Pick top K directions
 */
std::vector<Eigen::Vector3d> VisualHullConstructor::pickTopViewDirections(
    int k, SurfaceMesh& Mi) {
  std::unordered_map<int, std::vector<int>> regions =
      groupTrianglesIntoRegions(Mi);
  std::vector<int> regionHeadInfo;
  for (auto& kv : regions) {
    regionHeadInfo.push_back(kv.first);
  };

  auto weightedViewDirections =
      fitPlanesFromRegions(regionHeadInfo, regions, Mi);

  std::vector<Eigen::Vector3d> finalViewDirections =
      generateViewDirections(weightedViewDirections, k);

  std::cout << "view directions: " << finalViewDirections.size() << std::endl;
  return finalViewDirections;
}

std::vector<Eigen::Vector3d> VisualHullConstructor::getViewDirections() {
  return std::move(_topKDirections);
}

}  // namespace LowpolyGen