#include "engine/Generator.h"

namespace LowpolyGen {

LowpolyGen::SurfaceMesh readMeshFromOBJ(const std::string& OBJPath) {
  LowpolyGen::SurfaceMesh result{};
  // read input mesh(Mi)
  // Robustly read OBJ files. reference:
  // https://github.com/CGAL/cgal/issues/4062
  std::vector<typename Kernel::Point_3> points_ref;
  std::vector<std::vector<std::size_t>> faces_ref;
  std::ifstream in(OBJPath);
  if (!in || !CGAL::IO::read_OBJ(in, points_ref, faces_ref)) {
    std::cerr << "[LowpolyGen]: cannot read file." << std::endl;
    return {};
  }
  CGAL::Polygon_mesh_processing::orient_polygon_soup(
      points_ref,
      faces_ref);  // optional if your mesh is not correctly oriented
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(
      points_ref, faces_ref, result);
  return std::move(result);
}

Generator::Generator(const LowpolyGen::Config& conf) : _conf(conf){};

std::vector<LowpolyGen::SurfaceMesh> Generator::run(std::string& meshPath) {
  SurfaceMesh Mi = readMeshFromOBJ(meshPath);
  std::cout << "vertices: " << Mi.number_of_vertices() << std::endl;
  std::cout << "faces: " << Mi.number_of_faces() << std::endl;

  VisualHullConstructor vhc(_conf);
  SurfaceMesh Mv = vhc.run(Mi);

  std::vector<Eigen::Vector3d> K = vhc.getViewDirections();
  CarvedMeshGenerator cmg(K, _conf);
  SurfaceMesh Mc = cmg.run(Mi, Mv, _conf.N, _conf.epsilonTau);

  std::vector<SurfaceMesh> paretoSet = calculateParetoSet(Mc, _conf.T);
  return paretoSet;
}

}  // namespace LowpolyGen
