#ifndef _CALCULATE_TAU_UTIL_
#define _CALCULATE_TAU_UTIL_

#ifndef _COMMON_H_
#include "Common.h"
#endif  // _COMMON_H_

#include <renderer.h>

#include "mathUtil.h"

namespace LowpolyGen {

template <typename K1, typename K2>
double calculateTau(const CGAL::Surface_mesh<CGAL::Point_3<K1>>& Mi,
                    const CGAL::Surface_mesh<CGAL::Point_3<K2>>& Mo, double l) {
  double tau = 0.0;
  constexpr int kNumOfSamples = 100;
  int windowWidth = 128;
  int windowHeight = 128;

  glfwInit();
  GLFWwindow* window = glfwCreateWindow(windowWidth, windowHeight,
                                        "calculate tau", nullptr, nullptr);
  OpenGLRenderer renderer(window, windowWidth, windowHeight);

  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::exception("[OPENGL RENDERER]: Failed to initialize GLAD");
  }

  Shader vertexShaderInstanced =
      readShaderFromSource("resources/shaders/shaderVert.glsl");
  Shader fragmentShader =
      readShaderFromSource("resources/shaders/shaderFrag.glsl");
  renderer.addShader(ShaderType::VERTEX, vertexShaderInstanced, 0);
  renderer.addShader(ShaderType::FRAGMENT, fragmentShader, 0);

  // uniformly sample view directions on S^2(sphere)
  // for each direction, render Mi and Mo to stencil buffer
  // compute XOR of two stencil buffer
  // compute sqrt-average score
  // add score to total

  std::vector<Eigen::Vector3f> MiVerts;
  std::vector<Eigen::Vector3f> MoVerts;

  CGAL::Surface_mesh<CGAL::Point_3<K1>>::Halfedge_iterator hitForMi;
  for (hitForMi = Mi.halfedges_begin(); hitForMi != Mi.halfedges_end();
       ++hitForMi) {
    for (CGAL::Surface_mesh<CGAL::Point_3<K1>>::Vertex_index vd :
         Mi.vertices_around_face(*hitForMi)) {
      CGAL::Point_3<K1> pt = Mi.point(vd);
      float x = static_cast<float>(CGAL::to_double(pt.x()));
      float y = static_cast<float>(CGAL::to_double(pt.y()));
      float z = static_cast<float>(CGAL::to_double(pt.z()));
      Eigen::Vector3f ptFloat(x, y, z);
      MiVerts.push_back(ptFloat);
    }
  }

  CGAL::Surface_mesh<CGAL::Point_3<K2>>::Halfedge_iterator hitForMo;
  for (hitForMo = Mo.halfedges_begin(); hitForMo != Mo.halfedges_end();
       ++hitForMo) {
    for (CGAL::Surface_mesh<CGAL::Point_3<K2>>::Vertex_index vd :
         Mo.vertices_around_face(*hitForMo)) {
      CGAL::Point_3<K2> pt = Mo.point(vd);
      float x = static_cast<float>(CGAL::to_double(pt.x()));
      float y = static_cast<float>(CGAL::to_double(pt.y()));
      float z = static_cast<float>(CGAL::to_double(pt.z()));
      Eigen::Vector3f ptFloat(x, y, z);
      MoVerts.push_back(ptFloat);
    }
  }

  MeshModel mesh1(MiVerts);
  MeshModel mesh2(MoVerts);
  std::vector<MeshModel> modelVec1{mesh1};
  std::vector<MeshModel> modelVec2{mesh2};

  // std::vector<double> scoreArray(kNumOfSamples);

  std::vector<Eigen::Vector3f> pointsOnSphere(kNumOfSamples);

  for (int i = 0; i < kNumOfSamples; i++) {
    pointsOnSphere[i] = getHemiSpherePoint();
  }

  Scene scene1;
  scene1.loadMesh(modelVec1);

  Scene scene2;
  scene2.loadMesh(modelVec2);

  std::vector<std::vector<GLubyte>> RsOfMiArray(kNumOfSamples);
  std::vector<std::vector<GLubyte>> RsOfMoArray(kNumOfSamples);

  for (int cnt = 0; cnt < kNumOfSamples; cnt++) {
    Camera camera(toRadians(46), windowWidth / windowHeight, 0.01, 1000);
    Eigen::Vector3f ds = pointsOnSphere[cnt];

    // camera up 定义为与旋转轴平行
    Eigen::Vector3f beforeRotation(0, 0, 1);
    Eigen::Vector3f cameraUp = ds.cross(beforeRotation);
    cameraUp.normalize();
    Eigen::Vector3f cameraRight = cameraUp.cross(ds);
    cameraRight.normalize();

    ds *= 1.5 * l;

    camera.setDirection(ds, cameraUp, cameraRight);

    RsOfMiArray[cnt] = renderer.snapshot(scene1, camera);
    RsOfMoArray[cnt] = renderer.snapshot(scene2, camera);
  }

  // compute XOR
  for (int cnt = 0; cnt < kNumOfSamples; cnt++) {
    double score = 0.0;
    for (int i = 0; i < windowWidth * windowHeight; i++) {
      GLubyte vi = RsOfMiArray[cnt][i];
      GLubyte vo = RsOfMoArray[cnt][i];

      GLubyte res = vi ^ vo;

      score += std::abs(static_cast<double>(res));
    }
    score /= windowWidth * windowHeight;

    // scoreArray[cnt] = score;

    tau += score;
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return tau;
}
}  // namespace LowpolyGen
#endif
