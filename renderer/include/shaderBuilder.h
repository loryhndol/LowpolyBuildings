#pragma once
#include <glad/glad.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

enum class ShaderType {
  VERTEX,
  FRAGMENT,
  UNDEFINED,
};

class Shader {
 public:
  std::string filePath = "";
  std::string code = "";
  GLuint ID = 0;
  bool compiled = false;
};

Shader readShaderFromSource(const char* path);

class ShaderBuilder {
 private:
  ShaderBuilder() = default;
  ShaderBuilder(const ShaderBuilder&) = delete;
  static ShaderBuilder* pInstance;

 public:
  static ShaderBuilder* getBuilder();
  void compile(Shader& s, ShaderType t);
  ~ShaderBuilder();
};
