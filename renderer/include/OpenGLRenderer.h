#pragma once
#define _USE_MATH_DEFINES
#ifndef _USE_GLAD
#define _USE_GLAD
#include <glad/glad.h>
#endif

#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "camera.h"
#include "scene.h"
#include "shaderBuilder.h"

float toRadians(float degree);

class OpenGLRenderer {
 private:
  GLFWwindow* context;
  std::vector<GLuint> VAO;
  std::vector<GLuint> VBO;
  int windowWidth;
  int windowHeight;

  std::vector<std::unordered_map<ShaderType, Shader> >
      shaders;  // 记录每个模型的着色器

 public:
  OpenGLRenderer(GLFWwindow* window, int width, int height);
  ~OpenGLRenderer();
  void render(Scene& scene, Camera& camera);
  std::vector<GLubyte> snapshot(Scene& scene, Camera& camera);
  std::vector<GLubyte> snapshotV2(Scene& scene, Camera& camera, int modelIdx,
                                  int shaderIdx);
  std::vector<GLubyte> snapshotInstanced(Scene& scene,
                                         std::vector<Camera>& cameraArray,
                                         std::vector<Eigen::Vector2f>& offsets,
                                         int modelIdx, int shaderIdx,
                                         float scaleFactor);
  void addShader(const char* path, ShaderType t, int id);
  void addShader(ShaderType t, Shader s, int id);
  void bindWindow(GLFWwindow* window);
  void clearShader();
};
