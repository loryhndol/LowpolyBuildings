#pragma once

#include "engine/calculateTau.h"

namespace LowpolyGen {
float get_random(unsigned int seed = 0) {
  static std::default_random_engine e(seed);
  static std::uniform_real_distribution<float> u(-1, 1);
  return u(e);
}

Eigen::Vector3f getHemiSpherePoint() {
  float u = get_random(time(NULL));
  float v = get_random(time(NULL));
  float r2 = u * u + v * v;
  while (r2 >= 1) {
    u = get_random(time(NULL));
    v = get_random(time(NULL));
    r2 = u * u + v * v;
  }
  float z = std::sqrt(1 - r2);
  float phi = 2 * M_PI * u;
  float x = std::cos(phi) * std::sqrt(r2);
  float y = std::sin(phi) * std::sqrt(r2);
  return Eigen::Vector3f(x, y, z);
}

double calculateTau(const SurfaceMesh& Mi, const SurfaceMesh& Mo, double l) {
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

  typename SurfaceMesh::Halfedge_iterator hit;
  for (hit = Mi.halfedges_begin(); hit != Mi.halfedges_end(); ++hit) {
    for (typename SurfaceMesh::Vertex_index vd :
         Mi.vertices_around_face(*hit)) {
      CGAL::Point_3<Kernel> pt = Mi.point(vd);
      float x = static_cast<float>(CGAL::to_double(pt.x()));
      float y = static_cast<float>(CGAL::to_double(pt.y()));
      float z = static_cast<float>(CGAL::to_double(pt.z()));
      Eigen::Vector3f ptFloat(x, y, z);
      MiVerts.push_back(ptFloat);
    }
  }

  for (hit = Mo.halfedges_begin(); hit != Mo.halfedges_end(); ++hit) {
    for (typename SurfaceMesh::Vertex_index vd :
         Mo.vertices_around_face(*hit)) {
      CGAL::Point_3<Kernel> pt = Mo.point(vd);
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
