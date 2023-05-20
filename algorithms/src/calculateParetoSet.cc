#include "engine/calculateParetoSet.h"

namespace LowpolyGen {
std::vector<LowpolyGen::SurfaceMesh> calculateParetoSet(
    const LowpolyGen::SurfaceMesh& mesh) {
  std::vector<SurfaceMesh> meshArray;

  while (mesh.number_of_faces() > 4) {
    // keep 90% of edges
    double stop_ratio = 0.9;
    CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<
        CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>
        stop(stop_ratio);

    int r = CGAL::Surface_mesh_simplification::edge_collapse(mesh, stop);

    // edge - flip if any pair of adjacent triangles has an obtuse dihedral
    // angle larger than or if the exterior dihedral angle is smaller than
    // typename SurfaceMesh::edge_iterator eit;

    std::vector<typename SurfaceMesh::Halfedge_index> toBeFlipped;

    typename SurfaceMesh::halfedge_iterator hit;
    for (hit = mesh.halfedges_begin(); hit != mesh.halfedges_end(); ++hit) {
      typename SurfaceMesh::Halfedge_index oppositeHe = mesh.opposite(*hit);

      // calculate dihedral angle with normals
      CGAL::Vector_3<Kernel> n1 =
          CGAL::Polygon_mesh_processing::compute_face_normal(mesh.face(*hit),
                                                             mesh);
      CGAL::Vector_3<Kernel> n2 =
          CGAL::Polygon_mesh_processing::compute_face_normal(
              mesh.face(oppositeHe), mesh);

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
      CGAL::Euler::flip_edge(he, mesh);
    }

    repairMesh(mesh);
    if (mesh.number_of_faces() < maxTolerantElementsCnt) {
      meshArray.push_back(mesh);
    }

    if (r == 0) {
      break;
    }
  }

  // render to stencil buffer
  const int kWindowWidth = 128 * 16;
  const int kWindowHeight = 128 * 16;

  glfwInit();

  GLFWwindow* window = glfwCreateWindow(kWindowWidth, kWindowHeight,
                                        "stencil buffer", NULL, NULL);

  if (window == NULL) {
    throw std::exception("[OPENGL RENDERER]: Failed to create GLFW window");
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::exception("[OPENGL RENDERER]: Failed to initialize GLAD");
  }

  OpenGLRenderer instanceRenderer(window, kWindowWidth, kWindowHeight);
  Shader vertexShaderInstanced =
      readShaderFromSource("resources/shaders/shaderVertInstanced.glsl");
  Shader fragmentShader =
      readShaderFromSource("resources/shaders/shaderFrag.glsl");
  instanceRenderer.addShader(ShaderType::VERTEX, vertexShaderInstanced, 0);
  instanceRenderer.addShader(ShaderType::FRAGMENT, fragmentShader, 0);

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

  double l = (maxCoords - minCoords).norm();

  std::vector<SurfaceMesh> paretoSet;
  std::vector<double> visualMesure;
  for (int i = 0; i < meshArray.size(); i++) {
    double tau = calculateTau<Kernel>(meshArray[i], Mi, l, instanceRenderer,
                                      kWindowWidth, kWindowHeight);
    visualMesure.push_back(std::exp(-tau));
  }

  glfwDestroyWindow(window);
  glfwTerminate();

  int curIdx = 0;
  paretoSet.push_back(meshArray[0]);
  for (int i = 1; i < meshArray.size(); i++) {
    if (visualMesure[i] > visualMesure[curIdx]) {
      paretoSet.push_back(meshArray[i]);
      curIdx = i;
    }
  }

  return paretoSet;
}

}  // namespace LowpolyGen