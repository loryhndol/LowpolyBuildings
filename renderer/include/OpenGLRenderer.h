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
      shaders;  // shaders for each model

 public:
  OpenGLRenderer(GLFWwindow* window, int width, int height);
  ~OpenGLRenderer();
  std::vector<GLubyte> snapshot(Scene& scene, Camera& camera);
  void addShader(const char* path, ShaderType t, int id);
  void addShader(ShaderType t, Shader s, int id);
  void bindWindow(GLFWwindow* window);
  void clearShader();
};
